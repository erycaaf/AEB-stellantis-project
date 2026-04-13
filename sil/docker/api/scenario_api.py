"""
Scenario Control REST API
=========================
FastAPI server that bridges HTTP requests to ROS 2 services/topics
for controlling AEB simulation scenarios.

Runs inside a container with ROS 2 sourced.

Endpoints:
  GET  /scenarios          — list available scenarios
  POST /scenarios/start    — start/restart a scenario by name
  POST /scenarios/stop     — pause the simulation
  POST /scenarios/restart  — restart the current scenario
  GET  /status             — current scenario status
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import yaml
import subprocess
import threading
import os

app = FastAPI(title="AEB Scenario Control", version="1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Load scenario definitions ──────────────────────────────────────────────

SCENARIOS_FILE = "/workspace/aeb_gazebo/config/scenarios.yaml"

def load_scenarios():
    try:
        with open(SCENARIOS_FILE, "r") as f:
            data = yaml.safe_load(f)
            return data.get("scenarios", {})
    except FileNotFoundError:
        return {}

scenarios = load_scenarios()

# ── State ──────────────────────────────────────────────────────────────────

current_scenario = {"name": None, "status": "idle"}
lock = threading.Lock()


class StartRequest(BaseModel):
    scenario: str


def ros2_pub(topic, msg_type, data_str):
    """Publish a single message to a ROS 2 topic via CLI."""
    cmd = [
        "ros2", "topic", "pub", "--once",
        topic, msg_type, data_str
    ]
    subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env={**os.environ, "ROS_DOMAIN_ID": "0"}
    )


def ros2_service_call(service, srv_type, data_str="{}"):
    """Call a ROS 2 service via CLI."""
    cmd = [
        "ros2", "service", "call",
        service, srv_type, data_str
    ]
    subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env={**os.environ, "ROS_DOMAIN_ID": "0"}
    )


# ── Endpoints ──────────────────────────────────────────────────────────────

@app.get("/scenarios")
def list_scenarios():
    """List all available Euro NCAP scenarios."""
    result = []
    for key, cfg in scenarios.items():
        result.append({
            "id": key,
            "name": cfg.get("name", key),
            "ego_speed_kmh": cfg.get("ego_speed_kmh", 0),
            "target_speed_kmh": cfg.get("target_speed_kmh", 0),
            "initial_gap_m": cfg.get("initial_gap_m", 0),
            "pass_criterion": cfg.get("pass_criterion", ""),
        })
    return {"scenarios": result}


@app.post("/scenarios/start")
def start_scenario(req: StartRequest):
    """Start or restart a scenario by ID."""
    if req.scenario not in scenarios:
        return {"error": f"Unknown scenario: {req.scenario}"}

    cfg = scenarios[req.scenario]

    with lock:
        current_scenario["name"] = req.scenario
        current_scenario["status"] = "starting"

    # Reset Gazebo world (reposition vehicles to initial poses)
    ros2_service_call(
        "/reset_world",
        "std_srvs/srv/Empty"
    )

    # Send restart signal to scenario_controller node
    # (resets internal state: timers, flags, speed commands)
    ros2_pub(
        "/aeb/restart",
        "std_msgs/msg/Float64",
        '{"data": 1.0}'
    )

    with lock:
        current_scenario["status"] = "running"

    return {
        "status": "started",
        "scenario": req.scenario,
        "config": cfg
    }


@app.post("/scenarios/stop")
def stop_scenario():
    """Pause the simulation."""
    # Publish zero velocity to both vehicles
    ros2_pub("/aeb/ego/cmd_vel", "geometry_msgs/msg/Twist",
             '{"linear": {"x": 0.0}}')
    ros2_pub("/aeb/target/cmd_vel", "geometry_msgs/msg/Twist",
             '{"linear": {"x": 0.0}}')

    with lock:
        current_scenario["status"] = "stopped"

    return {"status": "stopped"}


@app.post("/scenarios/restart")
def restart_scenario():
    """Restart the current scenario."""
    # Reset Gazebo world
    ros2_service_call(
        "/reset_world",
        "std_srvs/srv/Empty"
    )

    # Signal scenario_controller to restart
    ros2_pub(
        "/aeb/restart",
        "std_msgs/msg/Float64",
        '{"data": 1.0}'
    )

    with lock:
        current_scenario["status"] = "running"

    return {"status": "restarting"}


@app.get("/status")
def get_status():
    """Get current scenario status."""
    return {
        "scenario": current_scenario["name"],
        "status": current_scenario["status"],
    }


@app.get("/live")
def get_live_data():
    """Get live telemetry from the running scenario."""
    import json
    try:
        with open('/tmp/aeb_live.json', 'r') as f:
            return json.load(f)
    except Exception:
        return {
            "time": 0, "distance": 0, "speed_kmh": 0,
            "brake_pct": 0, "fsm_state": "OFF", "ttc": 10.0
        }
