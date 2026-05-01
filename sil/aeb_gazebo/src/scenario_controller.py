#!/usr/bin/env python3
"""
Scenario Controller Node
========================
Controls both ego and target vehicle velocities to execute
Euro NCAP CCR test scenarios (CCRs, CCRm, CCRb).

Reads brake commands from the Zephyr ECU via vcan0 (python-can).
Publishes velocity commands to Gazebo via ROS 2.

CAN RX from vcan0 (from Zephyr ECU):
  0x080  AEB_BrakeCmd   — brake request, pressure, mode
  0x200  AEB_FSMState   — state, alert level, brake active
  0x300  AEB_Alert      — visual/audible alerts

ROS 2 (Gazebo interface):
  Sub: /aeb/ego/odom, /aeb/target/odom
  Pub: /aeb/ego/cmd_vel, /aeb/target/cmd_vel, /aeb/scenario_status
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion  # noqa: F401
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math
import time
import threading
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_codec import decode_brake_cmd, decode_fsm_state, decode_alert
from can_tcp_client import TcpCanBus


# Scenario presets — mirrors the launch file's SCENARIOS dict so a
# /aeb/restart with a scenario name can update params without going
# through `ros2 param set`. Keep these in sync if either side changes.
SCENARIOS = {
    'ccrs_20':     {'ego': 20.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_30':     {'ego': 30.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_40':     {'ego': 40.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_50':     {'ego': 50.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrm':        {'ego': 50.0, 'target': 20.0, 'gap': 50.0,  'decel': 0.0,  'brake_t': 0.0},
    'ccrb_d2_g12': {'ego': 50.0, 'target': 50.0, 'gap': 12.0,  'decel': -2.0, 'brake_t': 2.0},
    'ccrb_d2_g40': {'ego': 50.0, 'target': 50.0, 'gap': 40.0,  'decel': -2.0, 'brake_t': 2.0},
    'ccrb_d6_g12': {'ego': 50.0, 'target': 50.0, 'gap': 12.0,  'decel': -6.0, 'brake_t': 2.0},
    'ccrb_d6_g40': {'ego': 50.0, 'target': 50.0, 'gap': 40.0,  'decel': -6.0, 'brake_t': 2.0},
}


class ScenarioController(Node):
    def __init__(self):
        super().__init__('scenario_controller')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('scenario', 'ccrs_40')
        self.declare_parameter('ego_speed_kmh', 40.0)
        self.declare_parameter('target_speed_kmh', 0.0)
        self.declare_parameter('initial_gap_m', 100.0)
        self.declare_parameter('target_decel', 0.0)
        self.declare_parameter('target_brake_time', 0.0)
        self.declare_parameter('skip_teleport', False)
        self.declare_parameter('aeb_enabled', True)

        self.scenario_name = self.get_parameter('scenario').value
        self.ego_speed_ms = self.get_parameter('ego_speed_kmh').value / 3.6
        self.target_speed_ms = self.get_parameter('target_speed_kmh').value / 3.6
        self.initial_gap = self.get_parameter('initial_gap_m').value
        self.target_decel = self.get_parameter('target_decel').value
        self.target_brake_time = self.get_parameter('target_brake_time').value
        self.skip_teleport = self.get_parameter('skip_teleport').value
        self.aeb_enabled = self.get_parameter('aeb_enabled').value

        # ── CAN bus (TCP virtual bus — read brake commands from Zephyr ECU)
        can_host = os.environ.get('CAN_TCP_HOST', 'canbus')
        can_port = int(os.environ.get('CAN_TCP_PORT', '29536'))
        self.bus = TcpCanBus(host=can_host, port=can_port)
        if self.bus.sock is not None:
            self.get_logger().info(f'CAN RX connected to {can_host}:{can_port}')
            self.bus.on_receive(self._can_rx_callback)
        else:
            self.get_logger().error(f'Failed to connect to CAN bus')

        # ── State ─────────────────────────────────────────────────────────
        self.ego_x = 0.0
        self.ego_vx = 0.0
        self.ego_vx_cmd = 0.0
        self.target_x = self.initial_gap
        self.target_vx = self.target_speed_ms
        self.target_vx_cmd = self.target_speed_ms
        self.brake_cmd_pct = 0.0
        self.fsm_state_name = 'STANDBY'
        self.start_time = None
        self.scenario_running = False
        self.scenario_ended = False
        self.collision_detected = False
        self.collision_time = None
        self.max_decel_achieved = 0.0
        self.ego_has_braked = False
        self.perceived_distance = 100.0
        self.odom_ready = False
        self.gazebo_ready = False
        self.target_teleported = False
        self.world_ready_time = None
        self.world_settle_s = 5.0

        # ── ROS 2 publishers ──────────────────────────────────────────────
        self.ego_cmd_pub = self.create_publisher(Twist, '/aeb/ego/cmd_vel', 10)
        self.target_cmd_pub = self.create_publisher(Twist, '/aeb/target/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/aeb/scenario_status', 10)

        # ── ROS 2 subscribers ─────────────────────────────────────────────
        self.create_subscription(Odometry, '/aeb/ego/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/aeb/target/odom', self.target_odom_cb, 10)
        # /aeb/restart carries an optional scenario name in the String data.
        # Empty string = restart current scenario (Reset button).
        self.create_subscription(String, '/aeb/restart', self.restart_cb, 10)
        self.create_subscription(Float64, '/aeb/stop', self.stop_cb, 10)

        # Gazebo service for teleporting vehicles
        self.set_state_client = self.create_client(
            SetEntityState, '/aeb/set_entity_state'
        )

        self.can_lock = threading.Lock()

        # ── Main loop at 100 Hz ───────────────────────────────────────────
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info(f'=== Scenario: {self.scenario_name} ===')
        self.get_logger().info(
            f'Ego: {self.ego_speed_ms * 3.6:.0f} km/h, '
            f'Target: {self.target_speed_ms * 3.6:.0f} km/h, '
            f'Gap: {self.initial_gap:.0f} m'
        )
        self.get_logger().info('Waiting for Gazebo + vcan0...')

    # ── CAN RX callback ──────────────────────────────────────────────────

    def _can_rx_callback(self, arb_id, data, dlc):
        """Called by TcpCanBus RX thread for each received frame."""
        with self.can_lock:
            if arb_id == 0x080:  # BrakeCmd
                d = decode_brake_cmd(bytes(data).ljust(8, b'\x00'))
                self.brake_cmd_pct = d['brake_pressure'] / 0.1 if d['brake_pressure'] > 0 else 0.0
                self.brake_cmd_pct = min(100.0, max(0.0, self.brake_cmd_pct))

            elif arb_id == 0x200:  # FSMState
                d = decode_fsm_state(bytes(data).ljust(8, b'\x00'))
                self.fsm_state_name = d['fsm_name']

            elif arb_id == 0x300:  # Alert
                pass

    # ── Odom callbacks ─────────────────────────────────────────────────────

    def ego_odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_vx = msg.twist.twist.linear.x
        if not self.odom_ready:
            self.odom_ready = True
            self.world_ready_time = time.time()
            self.get_logger().info('Odom received — waiting for world to settle...')

    def target_odom_cb(self, msg):
        self.target_x = msg.pose.pose.position.x
        self.target_vx = msg.twist.twist.linear.x

    def restart_cb(self, msg):
        # If the message carries a known scenario name, switch to it;
        # otherwise keep the currently-loaded preset (Reset semantics).
        requested = (msg.data or '').strip()
        if requested and requested in SCENARIOS:
            cfg = SCENARIOS[requested]
            self.scenario_name      = requested
            self.ego_speed_ms       = cfg['ego']     / 3.6
            self.target_speed_ms    = cfg['target']  / 3.6
            self.initial_gap        = cfg['gap']
            self.target_decel       = cfg['decel']
            self.target_brake_time  = cfg['brake_t']

        self.get_logger().info(
            f'=== RESTARTING: {self.scenario_name} '
            f'(ego={self.ego_speed_ms*3.6:.0f} km/h, '
            f'target={self.target_speed_ms*3.6:.0f} km/h, '
            f'gap={self.initial_gap:.0f} m) ==='
        )

        self.ego_vx_cmd = 0.0
        self.target_vx_cmd = self.target_speed_ms
        self.brake_cmd_pct = 0.0
        self.scenario_running = False
        self.scenario_ended = False
        self.collision_detected = False
        self.collision_time = None
        self.max_decel_achieved = 0.0
        self.ego_has_braked = False
        self.perceived_distance = self.initial_gap
        self.world_ready_time = time.time()
        self.publish_vel(self.ego_cmd_pub, 0.0)
        self.publish_vel(self.target_cmd_pub, 0.0)
        self.gazebo_ready = True

        # Always teleport the target to the new initial gap on restart —
        # the world spawns it at x=100, but CCRb scenarios use gap=12 or 40.
        self.target_teleported = False
        self._teleport_target()

    def stop_cb(self, msg):
        """Halt the running scenario — both vehicles stop, status frozen."""
        self.get_logger().info('=== STOP requested via /aeb/stop ===')
        self.scenario_running = False
        self.scenario_ended = True
        self.ego_vx_cmd = 0.0
        self.target_vx_cmd = 0.0
        self.publish_vel(self.ego_cmd_pub, 0.0)
        self.publish_vel(self.target_cmd_pub, 0.0)
        self.publish_status('STOPPED: user halt')

    # ── Teleport helpers ───────────────────────────────────────────────────

    def _set_entity(self, name, x, y=-1.85):
        """Send a single set_entity_state request (fire-and-forget).

        Twist is zeroed so the planar_move plugin doesn't carry residual
        velocity from the previous run.
        """
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose = Pose(
            position=Point(x=float(x), y=float(y), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        req.state.twist = Twist()
        return self.set_state_client.call_async(req)

    def _teleport_target(self):
        # Reset BOTH vehicles via set_entity_state — /reset_world doesn't
        # always re-zero the planar_move plugin's internal odom integrator,
        # so on restart the ego could stay near the previous run's end pose.
        self.get_logger().info(
            f'Resetting vehicles: ego x=0, target x={self.initial_gap:.1f}'
        )
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('set_entity_state not available — skipping teleport')
            self.target_teleported = True
            self.create_timer(2.0, self._start_scenario_once)
            return

        self._set_entity('ego_vehicle',    0.0)
        future = self._set_entity('target_vehicle', self.initial_gap)
        future.add_done_callback(self._teleport_done)

    def _teleport_done(self, future):
        self.target_teleported = True
        # Snap our internal odom-tracked positions so the controller's
        # first publish doesn't rely on the next odom callback.
        self.ego_x    = 0.0
        self.target_x = self.initial_gap
        self.create_timer(2.0, self._start_scenario_once)

    def _start_scenario_once(self):
        if not self.scenario_running and not self.scenario_ended:
            self.scenario_running = True
            self.start_time = time.time()
            self.get_logger().info('>>> Scenario STARTED <<<')

    # ── Main control loop ──────────────────────────────────────────────────

    def control_loop(self):
        if self.scenario_ended:
            if self.collision_detected and self.collision_time:
                if time.time() - self.collision_time < 2.0:
                    self.publish_vel(self.ego_cmd_pub, self.ego_vx_cmd)
                    return
            self.publish_vel(self.ego_cmd_pub, 0.0)
            self.publish_vel(self.target_cmd_pub, 0.0)
            return

        if not self.scenario_running:
            # Wait for world to settle
            if self.odom_ready and self.world_ready_time:
                if time.time() - self.world_ready_time < self.world_settle_s:
                    return

            if self.odom_ready and not self.gazebo_ready:
                self.gazebo_ready = True
                self.get_logger().info('World ready — initializing scenario')
                if self.skip_teleport:
                    self.target_teleported = True
                    self.create_timer(2.0, self._start_scenario_once)
                elif not self.target_teleported:
                    self._teleport_target()
            return

        elapsed = time.time() - self.start_time
        dt = 0.01

        # Target vehicle motion
        if self.target_decel != 0.0 and elapsed >= self.target_brake_time:
            self.target_vx_cmd = max(0.0, self.target_vx_cmd + self.target_decel * dt)
        else:
            self.target_vx_cmd = self.target_speed_ms
        self.publish_vel(self.target_cmd_pub, self.target_vx_cmd)

        # Ego: apply brake from Zephyr ECU (0-100% → 0-10 m/s²)
        with self.can_lock:
            brake_pct = self.brake_cmd_pct

        brake_decel = (brake_pct / 100.0) * 10.0

        if self.aeb_enabled and brake_pct > 1.0:
            self.ego_vx_cmd = max(0.0, self.ego_vx_cmd - brake_decel * dt)
            self.ego_has_braked = True
        elif self.ego_has_braked and self.ego_vx_cmd > self.target_vx + 0.1:
            # AEB has released but the ego is still closing on the target.
            # Gazebo has no friction model in this world, so simulate a
            # 2 m/s² coast-decel until the ego matches the target speed
            # (CCRs: target_vx=0 → coast to a complete stop;
            #  CCRm/CCRb: settle at the target's velocity).
            self.ego_vx_cmd = max(0.0, self.ego_vx_cmd - 2.0 * dt)
        elif not self.ego_has_braked:
            self.ego_vx_cmd = self.ego_speed_ms

        self.publish_vel(self.ego_cmd_pub, self.ego_vx_cmd)

        if brake_decel > self.max_decel_achieved:
            self.max_decel_achieved = brake_decel

        # Distance from odom (ground truth)
        distance = self.target_x - self.ego_x - 4.5
        ego_speed_kmh = self.ego_vx * 3.6
        self.perceived_distance = distance

        # End conditions
        if distance <= 1.0 and ego_speed_kmh > 5.0:
            self.collision_detected = True
            self.scenario_ended = True
            if self.collision_time is None:
                self.collision_time = time.time()
            self.get_logger().warn(f'!!! COLLISION at v_impact = {ego_speed_kmh:.1f} km/h !!!')
            self.publish_status(f'COLLISION: v_impact={ego_speed_kmh:.1f} km/h')

        elif distance <= 1.0 and ego_speed_kmh <= 5.0 and elapsed > 2.0:
            self.scenario_ended = True
            self.get_logger().info(f'*** NEAR STOP: gap={distance:.1f}m v={ego_speed_kmh:.1f}km/h ***')
            self.publish_status(f'STOPPED: gap={distance:.1f}m v={ego_speed_kmh:.1f}km/h')

        elif self.ego_vx_cmd < 0.15 and elapsed > 2.0:
            self.scenario_ended = True
            self.get_logger().info(f'*** STOPPED: distance remaining = {distance:.1f} m ***')
            self.publish_status(f'STOPPED: gap={distance:.1f}m')

        elif (self.ego_has_braked
              and abs(self.ego_vx_cmd - self.target_vx) < 0.4
              and brake_pct < 1.0
              and elapsed > 4.0):
            # CCRm/CCRb: AEB has matched the ego's speed to the target's
            # and released. Distance is now stable or growing — no more
            # threat, end the run cleanly instead of waiting 60 s.
            self.scenario_ended = True
            v_settled_kmh = self.ego_vx_cmd * 3.6
            self.get_logger().info(
                f'*** SETTLED: v_ego={v_settled_kmh:.1f}km/h '
                f'gap={distance:.1f}m ***'
            )
            self.publish_status(
                f'SETTLED: v={v_settled_kmh:.1f}km/h gap={distance:.1f}m'
            )

        elif elapsed > 60.0:
            self.scenario_ended = True
            self.publish_status('TIMEOUT')

        # Periodic log + live data file for dashboard
        if int(elapsed * 100) % 10 == 0:  # every 100ms
            with self.can_lock:
                fsm = self.fsm_state_name
            # Write live data for API/dashboard
            try:
                import json
                live = {
                    'time': round(elapsed, 1),
                    'distance': round(distance, 1),
                    'speed_kmh': round(ego_speed_kmh, 1),
                    'brake_pct': round(brake_pct, 0),
                    'fsm_state': fsm,
                    'ttc': round(distance / max(0.5, self.ego_vx) if self.ego_vx > 0.5 else 10.0, 1),
                }
                with open('/tmp/aeb_live.json', 'w') as f:
                    json.dump(live, f)
            except Exception:
                pass

        if int(elapsed * 100) % 100 == 0:
            with self.can_lock:
                fsm = self.fsm_state_name
            self.get_logger().info(
                f't={elapsed:.1f}s  d={distance:.1f}m  '
                f'v_ego={ego_speed_kmh:.1f}km/h  '
                f'brake={brake_pct:.0f}%  FSM={fsm}'
            )

    def publish_vel(self, publisher, speed_ms):
        msg = Twist()
        msg.linear.x = speed_ms
        publisher.publish(msg)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.bus is not None:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
