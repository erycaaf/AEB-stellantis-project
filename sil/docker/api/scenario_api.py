"""
Scenario Control REST API
=========================
FastAPI server that bridges HTTP requests to ROS 2 services/topics
for controlling AEB simulation scenarios, plus UDS diagnostics over
the SIL's TCP CAN relay.

Runs inside a container with ROS 2 sourced.

Endpoints:
  GET  /scenarios            — list available scenarios
  POST /scenarios/start      — start/restart a scenario by name
  POST /scenarios/stop       — pause the simulation
  POST /scenarios/restart    — restart the current scenario
  GET  /status               — current scenario status
  GET  /live                 — live telemetry snapshot

  GET  /uds/health           — UDS connection status
  GET  /uds/read_did/{did}   — ReadDataByIdentifier (0x22)
  GET  /uds/snapshot         — read all three DIDs in one call
  POST /uds/clear_dtc        — ClearDiagnosticInformation (0x14)
  POST /uds/routine_control  — RoutineControl (0x31), e.g. AEB enable
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import yaml
import subprocess
import threading
import os

from uds_client import (
    UDSClient, UDSError, UDSTimeout,
    DID_TTC, DID_FSM_STATE, DID_BRAKE_PRESS, RID_AEB_CONTROL,
)

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


# ═══════════════════════════════════════════════════════════════════════════
#  UDS Diagnostics  (FR-UDS-001..005)
# ═══════════════════════════════════════════════════════════════════════════

CANBUS_HOST = os.environ.get("CAN_TCP_HOST", "canbus")
CANBUS_PORT = int(os.environ.get("CAN_TCP_PORT", "29536"))

_uds_client = None
_uds_lock   = threading.Lock()


def _get_uds_client():
    """Lazy-construct the UDS client on first use so the api container
    starts even when canbus is still coming up. Reconnects after a
    drop on the next call."""
    global _uds_client
    with _uds_lock:
        if _uds_client is None:
            try:
                _uds_client = UDSClient(
                    host=CANBUS_HOST, port=CANBUS_PORT, response_timeout=0.5
                )
            except ConnectionError as e:
                raise HTTPException(
                    status_code=503,
                    detail=f"UDS client cannot reach canbus: {e}",
                )
        return _uds_client


def _reset_uds_client():
    """Drop the client so the next call rebuilds it (after a transport error)."""
    global _uds_client
    with _uds_lock:
        if _uds_client is not None:
            try:
                _uds_client.close()
            except Exception:
                pass
            _uds_client = None


def _parse_did(did_str: str) -> int:
    """Accept '0xF101', 'F101', or '61697'."""
    s = did_str.strip()
    try:
        return int(s, 16) if s.lower().startswith("0x") or any(
            c in s.lower() for c in "abcdef"
        ) else int(s)
    except ValueError:
        raise HTTPException(
            status_code=400,
            detail=f"invalid DID: {did_str!r} (expected 0xF101 or 61697)"
        )


class RoutineControlRequest(BaseModel):
    rid: int
    value: int


@app.get("/uds/health")
def uds_health():
    """Report whether the UDS client is connected to canbus."""
    return {
        "host":      CANBUS_HOST,
        "port":      CANBUS_PORT,
        "connected": _uds_client is not None,
    }


@app.get("/uds/read_did/{did}")
def uds_read_did(did: str):
    """ReadDataByIdentifier (0x22). DIDs supported by the ECU:
        0xF100  TTC value (s)
        0xF101  FSM state
        0xF102  Brake pressure (%)
    """
    did_val = _parse_did(did)
    client = _get_uds_client()
    try:
        return client.read_did(did_val)
    except UDSTimeout as e:
        _reset_uds_client()
        raise HTTPException(status_code=504, detail=str(e))
    except UDSError as e:
        raise HTTPException(
            status_code=502,
            detail={
                "message": str(e),
                "sid":     e.sid,
                "nrc":     e.nrc,
            },
        )


@app.get("/uds/snapshot")
def uds_snapshot():
    """Convenience endpoint — read all three live DIDs in one call.
    Used by the dashboard's UDS panel for periodic polling."""
    client = _get_uds_client()
    out = {}
    for label, did in (("ttc", DID_TTC),
                       ("fsm", DID_FSM_STATE),
                       ("brake", DID_BRAKE_PRESS)):
        try:
            out[label] = client.read_did(did)
        except UDSTimeout as e:
            _reset_uds_client()
            out[label] = {"error": "timeout", "detail": str(e)}
        except UDSError as e:
            out[label] = {
                "error": "uds_error",
                "detail": str(e),
                "sid":    e.sid,
                "nrc":    e.nrc,
            }
    return out


@app.post("/uds/clear_dtc")
def uds_clear_dtc():
    """ClearDiagnosticInformation (0x14) — clears all DTCs."""
    client = _get_uds_client()
    try:
        return client.clear_dtc()
    except UDSTimeout as e:
        _reset_uds_client()
        raise HTTPException(status_code=504, detail=str(e))
    except UDSError as e:
        raise HTTPException(
            status_code=502,
            detail={"message": str(e), "sid": e.sid, "nrc": e.nrc},
        )


@app.post("/uds/routine_control")
def uds_routine_control(req: RoutineControlRequest):
    """RoutineControl (0x31). For AEB enable/disable use RID 0x0301
    with value 0 (disable) or 1 (enable)."""
    client = _get_uds_client()
    try:
        return client.routine_control(req.rid, req.value)
    except UDSTimeout as e:
        _reset_uds_client()
        raise HTTPException(status_code=504, detail=str(e))
    except UDSError as e:
        raise HTTPException(
            status_code=502,
            detail={"message": str(e), "sid": e.sid, "nrc": e.nrc},
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
