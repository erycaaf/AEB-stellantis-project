#!/usr/bin/env python3
"""
AEB Dashboard Node (GUI) — DEPRECATED
=====================================

@deprecated since the SIL PR landed the web dashboard at
http://localhost:3000 (sil/docker/dashboard/index.html). The web
dashboard is the supported visualization for the SIL stack: it shows
the same telemetry plus an embedded Gazebo 3D view, and runs in any
browser without a desktop matplotlib backend.

This file is kept temporarily to avoid breaking
`aeb_with_dashboard.launch.py`, which still wires it in for
contributors who prefer a standalone GUI window. It will be removed
in a follow-up cleanup PR — do not extend it.

----

Real-time matplotlib dashboard. Shows speedometer, TTC, FSM state,
brake bar, and alert indicators.

Subscribes to CAN frames:
  /can/ego_vehicle    [AEB_EgoVehicle]   — speed, accel
  /can/radar_target   [AEB_RadarTarget]  — distance, TTC
  /can/brake_cmd      [AEB_BrakeCmd]     — brake pressure / mode
  /can/fsm_state      [AEB_FSMState]     — state name
  /can/alert          [AEB_Alert]        — visual / audible flags
  /aeb/scenario_status [String]          — pass/fail result

Layout:
  Speedometer | TTC bar     | Visual Alert
              | FSM State   | Audible Alert
              | Distance    | Brake Bar
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from aeb_gazebo.msg import AebEgoVehicle, AebRadarTarget, AebBrakeCmd, AebFsmState, AebAlert

import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import matplotlib.gridspec as gridspec
import numpy as np
import threading
import time
import signal


FSM_COLORS = {
    'OFF': '#666666', 'STANDBY': '#4CAF50', 'WARNING': '#FF9800',
    'BRAKE_L1': '#FF5722', 'BRAKE_L2': '#F44336', 'BRAKE_L3': '#D50000',
    'POST_BRAKE': '#2196F3',
}
FSM_LABEL = {
    'OFF': 'OFF', 'STANDBY': 'STANDBY', 'WARNING': 'WARNING',
    'BRAKE_L1': 'BRAKE L1\n2 m/s²', 'BRAKE_L2': 'BRAKE L2\n4 m/s²',
    'BRAKE_L3': 'BRAKE L3\n6 m/s²', 'POST_BRAKE': 'POST BRAKE',
}

# BrakeMode index → label (matches DBC VAL_)
BRAKE_MODE_NAMES = {
    0: 'Off', 1: 'Warning', 2: 'Partial L1',
    3: 'Moderate L2', 4: 'Full L3', 5: 'Post-Brake Hold',
}


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.data = dict(
            speed=0.0, tgt_speed=0.0, dist=100.0, ttc=10.0,
            brake=0.0, state='STANDBY', visual=False, audible=False,
            status='', alive_radar=0, alive_ego=0, brake_mode=0,
        )
        self.lock = threading.Lock()

        # CAN frame subscribers
        self.create_subscription(AebEgoVehicle,  '/can/ego_vehicle',   self._ego_cb,   10)
        self.create_subscription(AebRadarTarget, '/can/radar_target',  self._radar_cb, 10)
        self.create_subscription(AebBrakeCmd,    '/can/brake_cmd',     self._brake_cb, 10)
        self.create_subscription(AebFsmState,    '/can/fsm_state',     self._fsm_cb,   10)
        self.create_subscription(AebAlert,       '/can/alert',         self._alert_cb, 10)

        # Non-CAN: scenario result
        self.create_subscription(String, '/aeb/scenario_status', self._status_cb, 10)

        self.get_logger().info('Dashboard GUI started — listening on /can/* topics')

    # ── CAN frame decoders ──────────────────────────────────────────────────

    def _ego_cb(self, msg: AebEgoVehicle):
        with self.lock:
            self.data['speed'] = msg.vehicle_speed * 3.6   # m/s → km/h
            self.data['alive_ego'] = msg.alive_counter

    def _radar_cb(self, msg: AebRadarTarget):
        with self.lock:
            self.data['dist']       = msg.target_distance
            self.data['ttc']        = msg.ttc
            # target speed = ego_speed - relative_speed (both m/s)
            ego_ms = self.data['speed'] / 3.6
            self.data['tgt_speed']  = max(0.0, ego_ms - msg.relative_speed) * 3.6
            self.data['alive_radar'] = msg.alive_counter

    def _brake_cb(self, msg: AebBrakeCmd):
        with self.lock:
            # brake_pressure [bar] → % : pressure = pct * 0.1  →  pct = pressure / 0.1
            self.data['brake']      = float(msg.brake_pressure) / 0.1
            self.data['brake_mode'] = msg.brake_mode

    def _fsm_cb(self, msg: AebFsmState):
        with self.lock:
            self.data['state'] = msg.fsm_state_name if msg.fsm_state_name else 'STANDBY'

    def _alert_cb(self, msg: AebAlert):
        with self.lock:
            self.data['visual']  = msg.visual_active
            self.data['audible'] = msg.audible_active

    def _status_cb(self, msg: String):
        with self.lock:
            self.data['status'] = msg.data

    def snap(self):
        with self.lock:
            return dict(self.data)


# ── Drawing helpers (unchanged) ─────────────────────────────────────────────

def draw_speed(ax, v, mx=80):
    ax.clear(); ax.set_xlim(-1.4, 1.4); ax.set_ylim(-0.4, 1.5)
    ax.set_aspect('equal'); ax.axis('off')
    t = np.linspace(np.pi, 0, 100)
    ax.plot(np.cos(t), np.sin(t), color='#444', lw=10, solid_capstyle='round')
    f = min(1.0, max(0.0, v / mx))
    if f > 0.01:
        n = max(2, int(100 * f))
        ts = np.linspace(np.pi, np.pi * (1 - f), n)
        c = '#4CAF50' if f < 0.5 else ('#FF9800' if f < 0.75 else '#F44336')
        ax.plot(np.cos(ts), np.sin(ts), color=c, lw=14, solid_capstyle='round')
    for i in range(0, mx + 1, 10):
        a = np.pi * (1 - i / mx)
        ax.plot([.82*np.cos(a), np.cos(a)], [.82*np.sin(a), np.sin(a)], color='#888', lw=1.5)
        ax.text(.68*np.cos(a), .68*np.sin(a), str(i),
                ha='center', va='center', fontsize=9, color='white')
    a = np.pi * (1 - f)
    ax.plot([0, .7*np.cos(a)], [0, .7*np.sin(a)], color='#FF1744', lw=3, solid_capstyle='round')
    ax.plot(0, 0, 'o', color='#FF1744', ms=8)
    ax.text(0, -.18, f'{v:.0f}', ha='center', va='center',
            fontsize=32, fontweight='bold', color='white')
    ax.text(0, -.35, 'km/h', ha='center', va='center', fontsize=11, color='#aaa')


def draw_ttc(ax, ttc):
    ax.clear(); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    ax.add_patch(FancyBboxPatch((.02, .15), .96, .5,
                                boxstyle="round,pad=0.02", fc='#333', ec='#555'))
    f = min(1.0, max(0.0, ttc / 6.0))
    c = '#4CAF50' if ttc > 4 else ('#FF9800' if ttc > 3 else ('#FF5722' if ttc > 2 else '#D50000'))
    if f > .01:
        ax.add_patch(FancyBboxPatch((.04, .18), .92 * f, .44,
                                    boxstyle="round,pad=0.01", fc=c, ec='none'))
    ax.text(.5, .82, 'TIME TO COLLISION',
            ha='center', va='center', fontsize=10, color='#aaa', fontweight='bold')
    s = f'{ttc:.1f} s' if ttc < 10 else '> 10 s'
    ax.text(.5, .4, s, ha='center', va='center', fontsize=20, fontweight='bold', color='white')


def draw_fsm(ax, st):
    ax.clear(); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    c = FSM_COLORS.get(st, '#666')
    lb = FSM_LABEL.get(st, st)
    ax.add_patch(FancyBboxPatch((.02, .05), .96, .9,
                                boxstyle="round,pad=0.05", fc=c, ec='white', lw=2))
    ax.text(.5, .5, lb, ha='center', va='center',
            fontsize=16, fontweight='bold', color='white')


def draw_dist(ax, d, tgt, status):
    ax.clear(); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    ax.add_patch(FancyBboxPatch((.02, .05), .96, .9,
                                boxstyle="round,pad=0.03", fc='#1a1a2e', ec='#555'))
    dc = '#4CAF50' if d > 20 else ('#FF9800' if d > 5 else '#F44336')
    ax.text(.5, .72, f'{d:.1f} m', ha='center', va='center',
            fontsize=18, fontweight='bold', color=dc)
    ax.text(.5, .42, 'DISTANCE', ha='center', va='center', fontsize=9, color='#aaa')
    ax.text(.5, .2, f'Target: {tgt:.0f} km/h',
            ha='center', va='center', fontsize=9, color='#888')
    if status:
        sc = '#4CAF50' if 'STOPPED' in status else '#F44336'
        ax.text(.5, .02, status[:30], ha='center', va='center',
                fontsize=8, fontweight='bold', color=sc)


def draw_alert(ax, vis, aud):
    ax.clear(); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    vc = '#FFD600' if vis else '#333'
    ax.add_patch(FancyBboxPatch((.05, .52), .42, .42,
                                boxstyle="round,pad=0.04", fc=vc, ec='#555', lw=2))
    tc = '#333' if vis else '#666'
    ax.text(.26, .73, '!', ha='center', va='center',
            fontsize=24, fontweight='bold', color=tc)
    ax.text(.26, .55, 'VISUAL', ha='center', va='center',
            fontsize=7, fontweight='bold', color=tc)
    ac = '#FF5722' if aud else '#333'
    ax.add_patch(FancyBboxPatch((.53, .52), .42, .42,
                                boxstyle="round,pad=0.04", fc=ac, ec='#555', lw=2))
    sym = ')))' if aud else '))'
    fc2 = 'white' if aud else '#666'
    ax.text(.74, .73, sym, ha='center', va='center',
            fontsize=18, fontweight='bold', color=fc2)
    ax.text(.74, .55, 'BUZZER', ha='center', va='center',
            fontsize=7, fontweight='bold', color=fc2)


def draw_brake(ax, pct):
    ax.clear(); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    ax.add_patch(FancyBboxPatch((.1, .05), .8, .75,
                                boxstyle="round,pad=0.02", fc='#333', ec='#555'))
    f = min(1.0, max(0.0, pct / 100.0))
    if f > .01:
        c = '#FF9800' if f < .5 else '#F44336'
        ax.add_patch(FancyBboxPatch((.13, .08), .74, .69 * f,
                                    boxstyle="round,pad=0.01", fc=c, ec='none'))
    ax.text(.5, .92, 'BRAKE', ha='center', va='center',
            fontsize=10, color='#aaa', fontweight='bold')
    ax.text(.5, .84, f'{pct:.0f}%', ha='center', va='center',
            fontsize=14, fontweight='bold', color='white')


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(8.4, 3), facecolor='#16213e')
    fig.canvas.manager.set_window_title('AEB Dashboard')

    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.2, wspace=0.25,
                           left=0.01, right=0.99, top=0.95, bottom=0.03)
    ax_spd = fig.add_subplot(gs[:, 0:2])
    ax_ttc = fig.add_subplot(gs[0, 2])
    ax_fsm = fig.add_subplot(gs[1, 2])
    ax_dst = fig.add_subplot(gs[2, 2])
    ax_alt = fig.add_subplot(gs[0, 3])
    ax_brk = fig.add_subplot(gs[1:, 3])

    plt.ion()
    plt.show(block=False)

    signal.signal(signal.SIGINT, lambda *_: None)

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            d = node.snap()
            draw_speed(ax_spd, d['speed'])
            draw_ttc(ax_ttc, d['ttc'])
            draw_fsm(ax_fsm, d['state'])
            draw_dist(ax_dst, d['dist'], d['tgt_speed'], d['status'])
            draw_alert(ax_alt, d['visual'], d['audible'])
            draw_brake(ax_brk, d['brake'])
            fig.suptitle('AEB SYSTEM MONITOR', fontsize=13,
                         fontweight='bold', color='#e0e0e0', y=0.98)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.1)
    except Exception:
        pass

    plt.close('all')
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
