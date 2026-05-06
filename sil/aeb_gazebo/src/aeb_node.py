#!/usr/bin/env python3
"""
AEB Controller Node
====================
Implements the AEB decision logic as a ROS2 node, reading sensor data
from Gazebo and publishing brake commands.

This node implements:
  - TTC computation (FR-DEC-001)
  - 7-state FSM (FR-FSM-001..005)
  - Progressive braking (FR-BRK-001)
  - Driver alert signals (FR-ALR-001..004)

Subscribes to:
  /aeb/distance       - Distance to target [m]
  /aeb/ego_speed      - Ego speed [km/h]
  /aeb/target_speed   - Target speed [km/h]
  /aeb/ttc            - Time to collision [s]

Publishes:
  /aeb/brake_cmd      - Brake command [0-100%]
  /aeb/fsm_state      - Current FSM state name
  /aeb/alert_visual   - Visual alert active [bool as Float64: 0/1]
  /aeb/alert_audible  - Audible alert active [bool as Float64: 0/1]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from enum import IntEnum
import time


class AEBState(IntEnum):
    OFF = 0
    STANDBY = 1
    WARNING = 2
    BRAKE_L1 = 3
    BRAKE_L2 = 4
    BRAKE_L3 = 5
    POST_BRAKE = 6


# TTC thresholds [s]
TTC_WARNING = 4.0
TTC_BRAKE_L1 = 3.0
TTC_BRAKE_L2 = 2.2
TTC_BRAKE_L3 = 1.8
TTC_HYSTERESIS = 0.15

# Deceleration targets [m/s^2]
DECEL = {
    AEBState.OFF: 0.0,
    AEBState.STANDBY: 0.0,
    AEBState.WARNING: 0.0,
    AEBState.BRAKE_L1: 2.0,
    AEBState.BRAKE_L2: 4.0,
    AEBState.BRAKE_L3: 6.0,
    AEBState.POST_BRAKE: 5.0,  # 50% brake hold
}

# Speed range [km/h]
V_MIN_KMH = 10.0
V_MAX_KMH = 60.0

# Timing
WARNING_MIN_TIME = 0.8  # 800ms before braking (FR-ALR-003)
DEBOUNCE_TIME = 0.2     # 200ms for de-escalation (FR-FSM-004)
POST_BRAKE_HOLD = 2.0   # 2.0s hold (FR-BRK-005)


class AEBNode(Node):
    def __init__(self):
        super().__init__('aeb_node')

        # State
        self.state = AEBState.STANDBY
        self.distance = 100.0
        self.ego_speed_kmh = 0.0
        self.target_speed_kmh = 0.0
        self.ttc = 10.0
        self.warning_entry_time = 0.0
        self.debounce_start = 0.0
        self.debounce_target_state = None
        self.post_brake_start = 0.0

        # Publishers
        self.brake_pub = self.create_publisher(Float64, '/aeb/brake_cmd', 10)
        self.state_pub = self.create_publisher(String, '/aeb/fsm_state', 10)
        self.visual_pub = self.create_publisher(Float64, '/aeb/alert_visual', 10)
        self.audible_pub = self.create_publisher(Float64, '/aeb/alert_audible', 10)

        # CAN frame publishers (for scenario_controller and dashboard)
        from aeb_gazebo.msg import AebBrakeCmd, AebFsmState, AebAlert
        self.can_brake_pub = self.create_publisher(AebBrakeCmd, '/can/brake_cmd', 10)
        self.can_fsm_pub   = self.create_publisher(AebFsmState, '/can/fsm_state', 10)
        self.can_alert_pub = self.create_publisher(AebAlert,    '/can/alert',     10)

        # Subscribers — CAN frame topics from perception_node
        from aeb_gazebo.msg import AebRadarTarget, AebEgoVehicle
        self.create_subscription(AebRadarTarget, '/can/radar_target', self._radar_cb, 10)
        self.create_subscription(AebEgoVehicle,  '/can/ego_vehicle',  self._ego_cb,   10)

        # Legacy Float64 subscribers (kept for backward compatibility)
        self.create_subscription(Float64, '/aeb/distance', self.distance_cb, 10)
        self.create_subscription(Float64, '/aeb/ego_speed', self.ego_speed_cb, 10)
        self.create_subscription(Float64, '/aeb/target_speed', self.target_speed_cb, 10)
        self.create_subscription(Float64, '/aeb/ttc', self.ttc_cb, 10)

        # 100 Hz control loop
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('AEB Node started - FSM active')

    # ── CAN frame callbacks (from perception_node) ────────────────────────

    def _radar_cb(self, msg):
        self.distance = msg.target_distance
        self.ttc = msg.ttc
        self.ego_speed_kmh = self.ego_speed_kmh  # keep current
        # Compute target speed from relative speed
        ego_ms = self.ego_speed_kmh / 3.6
        self.target_speed_kmh = max(0.0, ego_ms - msg.relative_speed) * 3.6

    def _ego_cb(self, msg):
        self.ego_speed_kmh = msg.vehicle_speed * 3.6  # m/s -> km/h

    # ── Legacy Float64 callbacks ────────────────────────────────────────

    def distance_cb(self, msg):
        self.distance = msg.data

    def ego_speed_cb(self, msg):
        self.ego_speed_kmh = msg.data

    def target_speed_cb(self, msg):
        self.target_speed_kmh = msg.data

    def ttc_cb(self, msg):
        self.ttc = msg.data

    def control_loop(self):
        now = time.time()
        ttc = self.ttc
        v_ego = self.ego_speed_kmh
        dist = self.distance
        prev_state = self.state

        # --- Speed range check (FR-DEC-008) ---
        # Once braking has started, keep active even below 10 km/h until stopped
        in_range = V_MIN_KMH <= v_ego <= V_MAX_KMH
        is_braking = self.state in (AEBState.BRAKE_L1, AEBState.BRAKE_L2, AEBState.BRAKE_L3)

        # Don't de-escalate when close to target (braking is working, let it finish)
        close_to_target = dist < 30.0 and v_ego > 2.0

        # Stopped check: low speed while close
        is_stopped = v_ego < 2.0 and dist < 10.0

        # --- FSM transitions ---
        if self.state == AEBState.OFF:
            self.state = AEBState.STANDBY

        elif self.state == AEBState.STANDBY:
            if (in_range or is_braking) and ttc <= TTC_WARNING:
                self.state = AEBState.WARNING
                self.warning_entry_time = now

        elif self.state == AEBState.WARNING:
            if ttc <= TTC_BRAKE_L1 and (now - self.warning_entry_time) >= WARNING_MIN_TIME:
                self.state = AEBState.BRAKE_L1
                self.debounce_start = 0.0
            elif not close_to_target and (ttc > TTC_WARNING + TTC_HYSTERESIS or not in_range):
                if self._debounce_check(now, AEBState.STANDBY):
                    self.state = AEBState.STANDBY

        elif self.state == AEBState.BRAKE_L1:
            if is_stopped:
                self.state = AEBState.POST_BRAKE
                self.post_brake_start = now
            elif ttc <= TTC_BRAKE_L2:
                self.state = AEBState.BRAKE_L2
                self.debounce_start = 0.0
            elif not close_to_target and ttc > TTC_BRAKE_L1 + TTC_HYSTERESIS:
                if self._debounce_check(now, AEBState.WARNING):
                    self.state = AEBState.WARNING
                    self.warning_entry_time = now

        elif self.state == AEBState.BRAKE_L2:
            if is_stopped:
                self.state = AEBState.POST_BRAKE
                self.post_brake_start = now
            elif ttc <= TTC_BRAKE_L3:
                self.state = AEBState.BRAKE_L3
                self.debounce_start = 0.0
            elif not close_to_target and ttc > TTC_BRAKE_L2 + TTC_HYSTERESIS:
                if self._debounce_check(now, AEBState.BRAKE_L1):
                    self.state = AEBState.BRAKE_L1

        elif self.state == AEBState.BRAKE_L3:
            if is_stopped:
                self.state = AEBState.POST_BRAKE
                self.post_brake_start = now
            elif not close_to_target and ttc > TTC_BRAKE_L3 + TTC_HYSTERESIS:
                if self._debounce_check(now, AEBState.BRAKE_L2):
                    self.state = AEBState.BRAKE_L2

        elif self.state == AEBState.POST_BRAKE:
            if (now - self.post_brake_start) >= POST_BRAKE_HOLD:
                self.state = AEBState.STANDBY

        # --- Compute brake command ---
        decel_target = DECEL[self.state]
        # Map deceleration to brake percentage: 10 m/s^2 = 100%
        brake_cmd = min(100.0, max(0.0, (decel_target / 10.0) * 100.0))

        # --- Compute alert outputs ---
        visual = 1.0 if self.state in (AEBState.WARNING, AEBState.BRAKE_L1,
                                        AEBState.BRAKE_L2, AEBState.BRAKE_L3,
                                        AEBState.POST_BRAKE) else 0.0
        audible = 1.0 if self.state in (AEBState.WARNING, AEBState.BRAKE_L1,
                                         AEBState.BRAKE_L2, AEBState.BRAKE_L3) else 0.0

        # --- Publish legacy Float64 topics ---
        self._pub_float(self.brake_pub, brake_cmd)
        self._pub_float(self.visual_pub, visual)
        self._pub_float(self.audible_pub, audible)

        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        # --- Publish CAN frame topics ---
        from aeb_gazebo.msg import AebBrakeCmd, AebFsmState, AebAlert

        brake_msg = AebBrakeCmd()
        brake_msg.brake_pressure = brake_cmd * 0.1  # pct -> bar (bar = pct/10, encoded *0.1)
        brake_msg.brake_request = brake_cmd > 0.0
        brake_msg.brake_mode = int(self.state)
        self.can_brake_pub.publish(brake_msg)

        fsm_msg = AebFsmState()
        fsm_msg.fsm_state = int(self.state)
        fsm_msg.fsm_state_name = self.state.name
        fsm_msg.alert_level = int(decel_target > 0)
        fsm_msg.brake_active = brake_cmd > 0.0
        self.can_fsm_pub.publish(fsm_msg)

        alert_msg = AebAlert()
        alert_msg.visual_active = visual > 0.5
        alert_msg.audible_active = audible > 0.5
        self.can_alert_pub.publish(alert_msg)

        # Log state transitions
        if self.state != prev_state:
            self.get_logger().info(
                f'FSM: {prev_state.name} -> {self.state.name}  '
                f'TTC={ttc:.2f}s  brake={brake_cmd:.0f}%'
            )

    def _debounce_check(self, now, target_state):
        """Returns True if debounce period has elapsed for target_state."""
        if self.debounce_target_state != target_state:
            self.debounce_target_state = target_state
            self.debounce_start = now
            return False
        return (now - self.debounce_start) >= DEBOUNCE_TIME

    def _pub_float(self, publisher, value):
        msg = Float64()
        msg.data = value
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
