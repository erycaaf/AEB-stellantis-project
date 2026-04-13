#!/bin/bash
# AEB Simulation Launcher with GUI
# Provides a simple button interface to launch scenarios

source /opt/ros/humble/setup.bash
cd ~/aeb_ws
colcon build --packages-select aeb_gazebo 2>/dev/null
source install/setup.bash

python3 - <<'PYEOF'
import subprocess
import signal
import sys
import os

os.environ['QT_QPA_PLATFORM'] = 'xcb'

import tkinter as tk
from tkinter import ttk

class AEBLauncher:
    def __init__(self):
        self.proc = None
        self.root = tk.Tk()
        self.root.title("AEB Simulation Launcher")
        self.root.configure(bg='#1a1a2e')
        self.root.geometry("420x480")
        self.root.resizable(False, False)

        # Title
        tk.Label(self.root, text="AEB Simulation", font=("Helvetica", 18, "bold"),
                 fg='white', bg='#1a1a2e').pack(pady=(15, 5))
        tk.Label(self.root, text="Select a scenario to launch", font=("Helvetica", 10),
                 fg='#aaa', bg='#1a1a2e').pack(pady=(0, 15))

        # Status
        self.status_var = tk.StringVar(value="Ready")
        self.status = tk.Label(self.root, textvariable=self.status_var,
                               font=("Helvetica", 11), fg='#4CAF50', bg='#1a1a2e')
        self.status.pack(pady=(0, 10))

        # Scenario buttons
        scenarios = [
            ("CCRs — 20 km/h", "ccrs_20", "#2196F3"),
            ("CCRs — 30 km/h", "ccrs_30", "#2196F3"),
            ("CCRs — 40 km/h", "ccrs_40", "#2196F3"),
            ("CCRs — 50 km/h", "ccrs_50", "#2196F3"),
            ("CCRm — 50/20 km/h", "ccrm", "#FF9800"),
            ("CCRb — d=2, gap=12m", "ccrb_d2_g12", "#F44336"),
            ("CCRb — d=6, gap=12m", "ccrb_d6_g12", "#F44336"),
            ("CCRb — d=2, gap=40m", "ccrb_d2_g40", "#D50000"),
            ("CCRb — d=6, gap=40m", "ccrb_d6_g40", "#D50000"),
        ]

        frame = tk.Frame(self.root, bg='#1a1a2e')
        frame.pack(fill='x', padx=20)

        for label, scenario, color in scenarios:
            btn = tk.Button(frame, text=label, font=("Helvetica", 11),
                           bg=color, fg='white', activebackground=color,
                           activeforeground='white', relief='flat',
                           cursor='hand2', height=1,
                           command=lambda s=scenario, l=label: self.launch(s, l))
            btn.pack(fill='x', pady=2)

        # Stop button
        tk.Button(frame, text="STOP", font=("Helvetica", 12, "bold"),
                  bg='#333', fg='#F44336', activebackground='#555',
                  activeforeground='#F44336', relief='flat',
                  cursor='hand2', height=1,
                  command=self.stop).pack(fill='x', pady=(10, 0))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def launch(self, scenario, label):
        self.stop()
        self.status_var.set(f"Launching: {label}...")
        self.status.configure(fg='#FF9800')
        self.root.update()

        cmd = (f"bash -c 'source /opt/ros/humble/setup.bash && "
               f"source ~/aeb_ws/install/setup.bash && "
               f"ros2 launch aeb_gazebo aeb_with_dashboard.launch.py scenario:={scenario}'")
        self.proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)

        self.status_var.set(f"Running: {label}")
        self.status.configure(fg='#4CAF50')

    def stop(self):
        if self.proc is not None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
            self.proc = None
        # Also kill any stray gazebo processes
        subprocess.run("killall -9 gzserver gzclient 2>/dev/null",
                       shell=True, capture_output=True)
        import time; time.sleep(1)
        self.status_var.set("Stopped")
        self.status.configure(fg='#aaa')

    def on_close(self):
        self.stop()
        self.root.destroy()

AEBLauncher()
PYEOF
