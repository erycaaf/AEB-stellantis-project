#!/usr/bin/env python3
"""
AEB Dashboard (PyQt5)
=====================
Standalone PyQt5 dashboard window showing AEB gauges.
Place side-by-side with Gazebo for presentation.
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                              QVBoxLayout, QHBoxLayout, QProgressBar, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPalette, QColor
import threading


FSM_COLORS = {
    'OFF': '#666666', 'STANDBY': '#4CAF50', 'WARNING': '#FF9800',
    'BRAKE_L1': '#FF5722', 'BRAKE_L2': '#F44336', 'BRAKE_L3': '#D50000',
    'POST_BRAKE': '#2196F3',
}


class Signals(QObject):
    speed = pyqtSignal(float)
    ttc = pyqtSignal(float)
    dist = pyqtSignal(float)
    brake = pyqtSignal(float)
    state = pyqtSignal(str)
    visual = pyqtSignal(float)
    audible = pyqtSignal(float)
    status = pyqtSignal(str)
    tgt_speed = pyqtSignal(float)


class RosNode(Node):
    def __init__(self, sig):
        super().__init__('dashboard_overlay')
        self.sig = sig
        self.create_subscription(Float64, '/aeb/ego_speed', lambda m: sig.speed.emit(m.data), 10)
        self.create_subscription(Float64, '/aeb/ttc', lambda m: sig.ttc.emit(m.data), 10)
        self.create_subscription(Float64, '/aeb/distance', lambda m: sig.dist.emit(m.data), 10)
        self.create_subscription(Float64, '/aeb/brake_cmd', lambda m: sig.brake.emit(m.data), 10)
        self.create_subscription(String, '/aeb/fsm_state', lambda m: sig.state.emit(m.data), 10)
        self.create_subscription(Float64, '/aeb/alert_visual', lambda m: sig.visual.emit(m.data), 10)
        self.create_subscription(Float64, '/aeb/alert_audible', lambda m: sig.audible.emit(m.data), 10)
        self.create_subscription(String, '/aeb/scenario_status', lambda m: sig.status.emit(m.data), 10)
        self.create_subscription(Float64, '/aeb/target_speed', lambda m: sig.tgt_speed.emit(m.data), 10)
        self.get_logger().info('Dashboard overlay node started')


def make_label(text='', size=16, color='white', bold=True):
    lbl = QLabel(text)
    f = QFont('Arial', size)
    f.setBold(bold)
    lbl.setFont(f)
    lbl.setStyleSheet(f'color: {color}; background: transparent;')
    lbl.setAlignment(Qt.AlignCenter)
    return lbl


def make_bar(color='#4CAF50'):
    bar = QProgressBar()
    bar.setRange(0, 100)
    bar.setValue(0)
    bar.setFixedHeight(14)
    bar.setTextVisible(False)
    bar.setStyleSheet(f'''
        QProgressBar {{ background: #333; border-radius: 7px; border: 1px solid #555; }}
        QProgressBar::chunk {{ background: {color}; border-radius: 7px; }}
    ''')
    return bar


class DashboardWindow(QMainWindow):
    def __init__(self, sig):
        super().__init__()
        self.setWindowTitle('AEB Dashboard')
        self.setFixedSize(500, 650)
        self.setStyleSheet('background-color: #16213e;')

        central = QWidget()
        self.setCentralWidget(central)
        main = QVBoxLayout(central)
        main.setContentsMargins(15, 10, 15, 10)
        main.setSpacing(8)

        # Title
        title = make_label('AEB SYSTEM MONITOR', 14, '#88a8d8')
        main.addWidget(title)
        main.addWidget(self._line())

        # Speed (big)
        self.speed_val = make_label('0', 56, '#4CAF50')
        self.speed_unit = make_label('km/h', 12, '#888', bold=False)
        main.addWidget(self.speed_val)
        main.addWidget(self.speed_unit)
        main.addWidget(self._line())

        # TTC
        ttc_row = QHBoxLayout()
        ttc_row.addWidget(make_label('TTC', 11, '#888', bold=False))
        self.ttc_val = make_label('> 10s', 22, '#4CAF50')
        ttc_row.addWidget(self.ttc_val)
        main.addLayout(ttc_row)
        self.ttc_bar = make_bar('#4CAF50')
        self.ttc_bar.setValue(100)
        main.addWidget(self.ttc_bar)
        main.addWidget(self._line())

        # FSM State
        self.fsm_label = make_label('STANDBY', 22, 'white')
        self.fsm_label.setFixedHeight(55)
        self.fsm_label.setStyleSheet(
            'color: white; background: #4CAF50; border-radius: 10px; padding: 8px;')
        main.addWidget(self.fsm_label)
        main.addWidget(self._line())

        # Distance
        dist_row = QHBoxLayout()
        dist_row.addWidget(make_label('DISTANCE', 11, '#888', bold=False))
        self.dist_val = make_label('100.0 m', 20, 'white')
        dist_row.addWidget(self.dist_val)
        main.addLayout(dist_row)
        tgt_row = QHBoxLayout()
        tgt_row.addWidget(make_label('TARGET', 10, '#666', bold=False))
        self.tgt_val = make_label('0 km/h', 14, '#888')
        tgt_row.addWidget(self.tgt_val)
        main.addLayout(tgt_row)
        main.addWidget(self._line())

        # Brake
        brake_row = QHBoxLayout()
        brake_row.addWidget(make_label('BRAKE', 11, '#888', bold=False))
        self.brake_val = make_label('0%', 20, '#888')
        brake_row.addWidget(self.brake_val)
        main.addLayout(brake_row)
        self.brake_bar = make_bar('#FF9800')
        main.addWidget(self.brake_bar)
        main.addWidget(self._line())

        # Alerts
        alerts_row = QHBoxLayout()
        self.visual_lbl = make_label('VISUAL', 13, '#444')
        self.visual_lbl.setFixedSize(120, 45)
        self.visual_lbl.setStyleSheet(
            'color: #444; background: #2a2a2a; border-radius: 8px; border: 2px solid #444;')
        self.audible_lbl = make_label('BUZZER', 13, '#444')
        self.audible_lbl.setFixedSize(120, 45)
        self.audible_lbl.setStyleSheet(
            'color: #444; background: #2a2a2a; border-radius: 8px; border: 2px solid #444;')
        alerts_row.addStretch()
        alerts_row.addWidget(self.visual_lbl)
        alerts_row.addWidget(self.audible_lbl)
        alerts_row.addStretch()
        main.addLayout(alerts_row)

        # Status
        main.addWidget(self._line())
        self.status_lbl = make_label('', 13, '#888')
        self.status_lbl.setFixedHeight(30)
        main.addWidget(self.status_lbl)

        main.addStretch()

        # Connect signals
        sig.speed.connect(self.on_speed)
        sig.ttc.connect(self.on_ttc)
        sig.dist.connect(self.on_dist)
        sig.brake.connect(self.on_brake)
        sig.state.connect(self.on_state)
        sig.visual.connect(self.on_visual)
        sig.audible.connect(self.on_audible)
        sig.status.connect(self.on_status)
        sig.tgt_speed.connect(self.on_tgt)

    def _line(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet('color: #333;')
        return line

    def on_speed(self, v):
        c = '#4CAF50' if v < 30 else ('#FF9800' if v < 50 else '#F44336')
        self.speed_val.setText(f'{v:.0f}')
        self.speed_val.setStyleSheet(f'color: {c}; background: transparent;')

    def on_ttc(self, t):
        c = '#4CAF50' if t > 4 else ('#FF9800' if t > 3 else ('#FF5722' if t > 2 else '#D50000'))
        s = f'{t:.1f}s' if t < 10 else '> 10s'
        self.ttc_val.setText(s)
        self.ttc_val.setStyleSheet(f'color: {c}; background: transparent;')
        self.ttc_bar.setValue(min(100, int(t / 6.0 * 100)))
        self.ttc_bar.setStyleSheet(f'''
            QProgressBar {{ background: #333; border-radius: 7px; border: 1px solid #555; }}
            QProgressBar::chunk {{ background: {c}; border-radius: 7px; }}
        ''')

    def on_dist(self, d):
        c = '#4CAF50' if d > 20 else ('#FF9800' if d > 5 else '#F44336')
        self.dist_val.setText(f'{d:.1f} m')
        self.dist_val.setStyleSheet(f'color: {c}; background: transparent;')

    def on_brake(self, b):
        c = '#888' if b < 1 else ('#FF9800' if b < 50 else '#F44336')
        self.brake_val.setText(f'{b:.0f}%')
        self.brake_val.setStyleSheet(f'color: {c}; background: transparent;')
        self.brake_bar.setValue(int(b))
        bc = '#FF9800' if b < 50 else '#F44336'
        self.brake_bar.setStyleSheet(f'''
            QProgressBar {{ background: #333; border-radius: 7px; border: 1px solid #555; }}
            QProgressBar::chunk {{ background: {bc}; border-radius: 7px; }}
        ''')

    def on_state(self, st):
        c = FSM_COLORS.get(st, '#666')
        self.fsm_label.setText(st.replace('_', ' '))
        self.fsm_label.setStyleSheet(
            f'color: white; background: {c}; border-radius: 10px; padding: 8px;')

    def on_visual(self, v):
        if v > 0.5:
            self.visual_lbl.setStyleSheet(
                'color: #333; background: #FFD600; border-radius: 8px; border: 2px solid #FFA000; font-weight: bold;')
        else:
            self.visual_lbl.setStyleSheet(
                'color: #444; background: #2a2a2a; border-radius: 8px; border: 2px solid #444;')

    def on_audible(self, a):
        if a > 0.5:
            self.audible_lbl.setStyleSheet(
                'color: white; background: #FF5722; border-radius: 8px; border: 2px solid #E64A19; font-weight: bold;')
        else:
            self.audible_lbl.setStyleSheet(
                'color: #444; background: #2a2a2a; border-radius: 8px; border: 2px solid #444;')

    def on_status(self, s):
        if 'STOPPED' in s or 'NEAR' in s:
            self.status_lbl.setStyleSheet('color: #4CAF50; background: rgba(76,175,80,30); border-radius: 5px;')
        elif 'COLLISION' in s:
            self.status_lbl.setStyleSheet('color: #F44336; background: rgba(244,67,54,30); border-radius: 5px;')
        else:
            self.status_lbl.setStyleSheet('color: #888; background: transparent;')
        self.status_lbl.setText(s)

    def on_tgt(self, v):
        self.tgt_val.setText(f'{v:.0f} km/h')


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(22, 33, 62))
    palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
    app.setPalette(palette)

    sig = Signals()
    node = RosNode(sig)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    win = DashboardWindow(sig)
    win.show()
    ret = app.exec_()

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(ret)


if __name__ == '__main__':
    main()
