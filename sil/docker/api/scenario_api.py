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
import time
import os

try:
    import docker as docker_sdk
    _docker_client = docker_sdk.from_env()
except Exception as _e:  # noqa: BLE001 — log but degrade gracefully
    print(f"[api] docker SDK unavailable ({_e!r}) — ECU restart disabled", flush=True)
    _docker_client = None

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


def _run_with_timeout(cmd, timeout=8):
    """Run a subprocess with a timeout, return (returncode, stdout, stderr).

    Uses Popen + communicate(timeout=...) directly — sidesteps a Python
    quirk in this image where subprocess.run forwards `timeout` to Popen.
    """
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env={**os.environ, "ROS_DOMAIN_ID": "0"},
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, stderr = proc.communicate()
        return -1, stdout, stderr
    return proc.returncode, stdout, stderr


def ros2_pub(topic, msg_type, data_str):
    """Publish a single message to a ROS 2 topic via CLI, synchronously."""
    cmd = ["ros2", "topic", "pub", "--once", topic, msg_type, data_str]
    try:
        rc, _stdout, stderr = _run_with_timeout(cmd, timeout=8)
        if rc != 0:
            print(
                f"[ros2_pub] non-zero exit ({rc}) for {topic}: "
                f"stderr={stderr.decode(errors='replace').strip()!r}",
                flush=True,
            )
        else:
            print(f"[ros2_pub] {topic} -> {data_str}", flush=True)
    except Exception as e:
        print(f"[ros2_pub] exception publishing to {topic}: {e!r}", flush=True)


def restart_aeb_ecu():
    """Restart the aeb-ecu container to clear latched perception state.

    The Zephyr ECU's `prev_d` / `prev_vr` ROC baselines are static state
    that only `perception_init()` clears, and that runs once at process
    startup. After a scenario teleport jumps the target distance by > 10 m
    (DIST_ROC_LIMIT), every subsequent radar/lidar frame is rejected as
    a ROC violation, fault_flag latches, and the FSM is stuck in OFF.
    Restarting the container is the cleanest way to get a fresh ECU.
    """
    if _docker_client is None:
        return
    try:
        ecu = _docker_client.containers.get("aeb-ecu")
        ecu.restart(timeout=2)
        # Give the new process ~1.5 s to come up and reconnect to canbus
        # before perception starts pumping CAN frames at it.
        time.sleep(1.5)
        print("[api] aeb-ecu container restarted", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[api] failed to restart aeb-ecu: {e!r}", flush=True)


def ros2_service_call(service, srv_type, data_str="{}"):
    """Call a ROS 2 service synchronously (e.g. /reset_world)."""
    cmd = ["ros2", "service", "call", service, srv_type, data_str]
    try:
        rc, _stdout, stderr = _run_with_timeout(cmd, timeout=8)
        if rc != 0:
            print(
                f"[ros2_service_call] non-zero exit ({rc}) for {service}: "
                f"stderr={stderr.decode(errors='replace').strip()!r}",
                flush=True,
            )
    except Exception as e:
        print(f"[ros2_service_call] exception calling {service}: {e!r}", flush=True)


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
    """Start or restart a scenario by ID.

    The scenario name is shipped to the controller via /aeb/restart; the
    controller has its own copy of the scenarios dict, so no `ros2 param
    set` chain is needed (that round-trip used to add ~3 s before the run
    even began, and the API/controller could race on it).
    """
    if req.scenario not in scenarios:
        return {"error": f"Unknown scenario: {req.scenario}"}

    cfg = scenarios[req.scenario]

    with lock:
        current_scenario["name"] = req.scenario
        current_scenario["status"] = "starting"

    # Restart the Zephyr ECU first — its perception fault watchdog latches
    # across runs (see restart_aeb_ecu docstring). Then reset Gazebo so the
    # controller's teleport doesn't race with the world reset.
    restart_aeb_ecu()
    ros2_service_call("/reset_world", "std_srvs/srv/Empty")

    # Then tell the controller which scenario to load and reset its state.
    # YAML flow style with unquoted key — the form `ros2 topic pub` parses
    # most reliably in our testing.
    ros2_pub(
        "/aeb/restart",
        "std_msgs/msg/String",
        f'{{data: "{req.scenario}"}}',
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
    """Halt the running scenario.

    Sends /aeb/stop so scenario_controller flips into the ended state and
    holds both vehicles at zero velocity. Without this signal, publishing
    cmd_vel=0 once is overridden 10 ms later by the controller's own loop.
    """
    ros2_pub(
        "/aeb/stop",
        "std_msgs/msg/Float64",
        '{"data": 1.0}'
    )

    with lock:
        current_scenario["status"] = "stopped"

    return {"status": "stopped"}


@app.post("/scenarios/restart")
def restart_scenario():
    """Restart the current scenario (no scenario change).

    Empty data field tells the controller to keep its existing preset.
    """
    restart_aeb_ecu()
    ros2_service_call("/reset_world", "std_srvs/srv/Empty")
    ros2_pub("/aeb/restart", "std_msgs/msg/String", '{data: ""}')

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
