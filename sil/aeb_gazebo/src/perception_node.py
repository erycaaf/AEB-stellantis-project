#!/usr/bin/env python3
"""
AEB Perception Node — Sensor Simulation via CAN
================================================
Reads ground-truth from Gazebo odom, applies sensor noise and fault
injection, encodes into DBC-format CAN frames, and writes directly
to vcan0 via python-can.

TX on vcan0 (to Zephyr ECU):
  0x100  AEB_EgoVehicle   @ 10 ms  — speed, accel, yaw, steering
  0x101  AEB_DriverInput  @ 10 ms  — pedals, AEB enable, override
  0x120  AEB_RadarTarget  @ 20 ms  — distance, rel_speed, TTC, confidence

Subscribes (ROS 2, from Gazebo):
  /aeb/ego/odom           — ego vehicle odometry
  /aeb/target/odom        — target vehicle odometry

Publishes (ROS 2, for scenario_controller + dashboard):
  /aeb/scenario_status    — (nothing, but keeps node in the graph)

Parameters:
  noise_sigma_d   — distance noise std dev [m] (default 0.25)
  noise_sigma_v   — velocity noise std dev [m/s] (default 0.1)
  fault_inject    — force sensor fault (0=normal, 1=fault)
  lidar_fail      — disable lidar sensor (0=ok, 1=fail)
  radar_fail      — disable radar sensor (0=ok, 1=fail)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
import time
import random
import sys
import os

# Add parent dir to path for local modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_codec import encode_ego_vehicle, encode_driver_input, encode_radar_target
from can_tcp_client import TcpCanBus

# ── Constants ──────────────────────────────────────────────────────────────

CAN_ID_EGO_VEHICLE  = 0x100
CAN_ID_DRIVER_INPUT = 0x101
CAN_ID_RADAR_TARGET = 0x120

EGO_LENGTH = 4.5            # m (bumper-to-bumper offset)
TTC_MAX    = 10.0
V_REL_MIN  = 0.5

# Sensor models
RADAR_RANGE_MIN = 0.5
RADAR_RANGE_MAX = 260.0
LIDAR_RANGE_MIN = 0.3
LIDAR_RANGE_MAX = 200.0
FAULT_CYCLE_LIMIT = 3

# Fusion weights
W_RADAR = 0.3
W_LIDAR = 0.7
CROSS_SENSOR_TOL = 5.0
DIST_RATE_MAX = 60.0


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ── Parameters (tunable for fault injection) ──────────────────────
        self.declare_parameter('noise_sigma_d', 0.25)
        self.declare_parameter('noise_sigma_v', 0.1)
        self.declare_parameter('fault_inject', 0)
        self.declare_parameter('lidar_fail', 0)
        self.declare_parameter('radar_fail', 0)

        # ── CAN bus (TCP virtual bus) ─────────────────────────────────────
        can_host = os.environ.get('CAN_TCP_HOST', 'canbus')
        can_port = int(os.environ.get('CAN_TCP_PORT', '29536'))
        self.bus = TcpCanBus(host=can_host, port=can_port)
        if self.bus.sock is not None:
            self.get_logger().info(f'CAN bus connected to {can_host}:{can_port}')
        else:
            self.get_logger().error(f'Failed to connect to CAN bus at {can_host}:{can_port}')

        # ── State ─────────────────────────────────────────────────────────
        self.ego_x = 0.0
        self.ego_vx = 0.0
        self.ego_accel = 0.0
        self.ego_vx_prev = 0.0
        self.last_accel_time = None
        self.target_x = 100.0
        self.target_vx = 0.0
        self.odom_ready = False

        # Sensor simulation state
        self.prev_fused_distance = 100.0
        self.last_fusion_time = None
        self.fault_counter = 0

        # Driver AEB-enable bit broadcast in CAN 0x101. Default 1 (AEB on);
        # the API publishes 0 to /aeb/driver_aeb_enable when starting a
        # `disable_aeb: true` scenario. The production C ECU's FSM
        # Priority-1 reads `state->driver.aeb_enabled`, which is overwritten
        # from this CAN frame every cycle — so toggling this bit at the
        # source is the only way to keep AEB off across cycles. (UDS
        # RoutineControl 0x0301 writes to state->uds.aeb_enabled, a
        # separate field not consumed by the FSM.)
        self.driver_aeb_enable = 1

        # ── ROS 2 subscriptions (from Gazebo) ─────────────────────────────
        self.create_subscription(Odometry, '/aeb/ego/odom', self.ego_odom_cb, 10)
        self.create_subscription(Odometry, '/aeb/target/odom', self.target_odom_cb, 10)
        self.create_subscription(Int32, '/aeb/driver_aeb_enable',
                                 self.aeb_enable_cb, 10)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(0.01, self.publish_ego_vehicle)     # 10 ms
        self.create_timer(0.01, self.publish_driver_input)    # 10 ms
        self.create_timer(0.02, self.publish_radar_target)    # 20 ms

        self.get_logger().info(
            'Perception node started — TX on vcan0: '
            '0x100 (10ms), 0x101 (10ms), 0x120 (20ms)'
        )

    # ── Odom callbacks ─────────────────────────────────────────────────────

    def ego_odom_cb(self, msg):
        now = time.time()
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        if self.last_accel_time is not None:
            dt = now - self.last_accel_time
            if dt > 0.001:
                self.ego_accel = (speed - self.ego_vx_prev) / dt
        self.ego_vx_prev = speed
        self.last_accel_time = now

        self.ego_x = msg.pose.pose.position.x
        self.ego_vx = speed
        if not self.odom_ready:
            self.odom_ready = True
            self.get_logger().info('Odom received — sensors active')

    def target_odom_cb(self, msg):
        self.target_x = msg.pose.pose.position.x
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.target_vx = math.sqrt(vx * vx + vy * vy)

    # ── Sensor simulation ──────────────────────────────────────────────────

    def _simulate_sensor(self, gt_dist, range_min, range_max, noise_std, fail_param):
        """Simulate one sensor reading with noise and fault injection.

        Noise model:
          - default: sigma = `noise_sigma_d` parameter (single global value
            applied identically to radar and lidar, so the Kalman fusion
            sees perfectly-correlated readings — useful for nominal runs).
          - CAN_SIL_NOISE=1 in the gazebo container env: sigma is the
            per-sensor `noise_std` argument (radar = 0.25 m, lidar =
            0.05 m), so the two sensors disagree by realistic amounts.
            That actually stresses the fusion's plausibility / cross-
            sensor-tolerance branches in `_fuse_sensors`.
        """
        fail = self.get_parameter(fail_param).value
        fi = self.get_parameter('fault_inject').value

        if fail or fi:
            return None

        if range_min <= gt_dist <= range_max:
            if os.environ.get('CAN_SIL_NOISE') == '1':
                sigma = noise_std
            else:
                sigma = self.get_parameter('noise_sigma_d').value
                if sigma <= 0:
                    sigma = noise_std
            return gt_dist + random.gauss(0, sigma)
        return None

    def _fuse_sensors(self, gt_dist):
        """Fuse radar + lidar with plausibility checks. Returns (distance, confidence)."""
        now = time.time()

        radar_d = self._simulate_sensor(gt_dist, RADAR_RANGE_MIN, RADAR_RANGE_MAX, 0.25, 'radar_fail')
        lidar_d = self._simulate_sensor(gt_dist, LIDAR_RANGE_MIN, LIDAR_RANGE_MAX, 0.05, 'lidar_fail')

        radar_ok = radar_d is not None and 0 <= radar_d <= 300
        lidar_ok = lidar_d is not None and 0 <= lidar_d <= 300

        raw_distance = None
        if radar_ok and lidar_ok:
            if abs(radar_d - lidar_d) > CROSS_SENSOR_TOL:
                raw_distance = lidar_d
            else:
                raw_distance = W_RADAR * radar_d + W_LIDAR * lidar_d
        elif lidar_ok:
            raw_distance = lidar_d
        elif radar_ok:
            raw_distance = radar_d

        # Rate-of-change check
        if raw_distance is not None and self.last_fusion_time is not None:
            dt = now - self.last_fusion_time
            if dt > 0.001:
                rate = abs(raw_distance - self.prev_fused_distance) / dt
                if rate > DIST_RATE_MAX:
                    raw_distance = None

        # Fault detection
        if raw_distance is not None:
            self.fault_counter = 0
            self.prev_fused_distance = raw_distance
            self.last_fusion_time = now
        else:
            self.fault_counter += 1

        # Confidence
        if self.fault_counter >= FAULT_CYCLE_LIMIT:
            confidence = 0   # Fault
        elif radar_ok and lidar_ok:
            confidence = 15  # BothSensors
        elif radar_ok or lidar_ok:
            confidence = 8   # SingleSensor
        else:
            confidence = 0   # Fault

        distance = raw_distance if raw_distance is not None else self.prev_fused_distance
        return max(0.0, min(300.0, distance)), confidence

    # ── CAN TX ─────────────────────────────────────────────────────────────

    def _can_send(self, arb_id, data):
        if self.bus is None or self.bus.sock is None:
            return
        self.bus.send(arb_id, data)

    def publish_ego_vehicle(self):
        if not self.odom_ready:
            return
        data = encode_ego_vehicle(
            speed_ms=self.ego_vx,
            accel_ms2=self.ego_accel,
        )
        self._can_send(CAN_ID_EGO_VEHICLE, data)

    def aeb_enable_cb(self, msg):
        """Latched override for the AEB-enable bit broadcast in CAN 0x101.
        Set by the API when the scenario YAML has `disable_aeb: true`."""
        self.driver_aeb_enable = 1 if msg.data else 0
        self.get_logger().info(
            f'driver_aeb_enable set to {self.driver_aeb_enable}')

    def publish_driver_input(self):
        if not self.odom_ready:
            return
        data = encode_driver_input(
            brake_pct=0,
            accel_pct=0,
            aeb_enable=self.driver_aeb_enable,
            override=0,
        )
        self._can_send(CAN_ID_DRIVER_INPUT, data)

    def publish_radar_target(self):
        if not self.odom_ready:
            return

        gt_dist = self.target_x - self.ego_x - EGO_LENGTH
        distance, confidence = self._fuse_sensors(gt_dist)

        v_rel = self.ego_vx - self.target_vx
        ttc = TTC_MAX
        if v_rel > V_REL_MIN and distance > 0:
            ttc = max(0.0, min(TTC_MAX, distance / v_rel))

        data = encode_radar_target(
            distance_m=distance,
            rel_speed_ms=v_rel,
            ttc_s=ttc,
            confidence=confidence,
        )
        self._can_send(CAN_ID_RADAR_TARGET, data)

    def destroy_node(self):
        if self.bus is not None:
            self.bus.shutdown()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
