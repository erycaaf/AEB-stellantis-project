# AEB SIL — Software-in-the-Loop Validation

End-to-end Software-in-the-Loop environment for the AEB system. Runs the
embedded C code (`src/`) inside a Zephyr `native_sim` ECU and exercises it
against a Gazebo Classic 11 vehicle simulation, with a TCP CAN bus relaying
frames between the perception, ECU, and scenario containers.

> **Status:** integrated and validated — CCRs at 38.8 km/h stops at 6.5 m,
> collision avoided, FSM sequence `STANDBY → WARNING → BRAKE_L1 → BRAKE_L2 →
> POST_BRAKE`. See [`reports/sil/`](../reports/sil/) for run evidence.

---

## Architecture

```
                                 ┌─────────────────────────┐
                                 │  TCP CAN bus (:29536)   │
                                 │       canbus            │
                                 └────────┬────────┬───────┘
                                          │        │
                  ┌───────────────────────┘        └───────────────────────┐
                  ▼                                                        ▼
        ┌──────────────────┐    ros2 topics    ┌──────────────────────────────┐
        │   gazebo         │◀─────────────────▶│  aeb-ecu (Zephyr native_sim) │
        │ ─ gzserver/client│                   │  ─ aeb_perception            │
        │ ─ perception_node│                   │  ─ aeb_ttc · aeb_fsm         │
        │ ─ scenario_ctl   │                   │  ─ aeb_pid · aeb_alert       │
        │ ─ dashboard_node │                   │  ─ aeb_can · aeb_uds         │
        │ ─ noVNC :6080    │                   │  10 ms cycle                 │
        └────────┬─────────┘                   └──────────────────────────────┘
                 │
                 ▼
        ┌──────────────────┐         ┌──────────────────┐
        │  api  (FastAPI)  │◀────────│ dashboard (nginx)│
        │  :8000           │         │ :3000            │
        └──────────────────┘         └──────────────────┘
```

**Data flow** (all CAN frames over TCP, port 29536):

```
perception_node ──► canbus ──► aeb-ecu ──► canbus ──► scenario_controller
                                  │
                                  └─► dashboard_node (state, alerts, brake %)
```

The Zephyr ECU runs the **same C code** that ships in this repository
(`src/perception/*`, `src/decision/*`, `src/execution/*`,
`src/communication/*`, `src/integration/aeb_core.c`). No reimplementation.

---

## Services

| Service     | Image                                | Purpose                                                                        |
|-------------|--------------------------------------|--------------------------------------------------------------------------------|
| `canbus`    | `python:3.11-slim`                   | TCP relay server fanning CAN frames between the other services on `:29536`.    |
| `gazebo`    | `osrf/ros:humble-desktop` (built)    | Gazebo 11 + ROS 2 Humble + the `aeb_gazebo` package (perception, scenario).    |
| `aeb-ecu`   | `ghcr.io/zephyrproject-rtos/ci`      | Zephyr `native_sim` running `aeb_core` end-to-end on a 10 ms cycle.            |
| `api`       | `osrf/ros:humble-desktop` (built)    | FastAPI scenario-control endpoints (`:8000`) used by the dashboard.            |
| `dashboard` | `nginx:alpine`                       | Static dashboard at `:3000` (noVNC viewer + scenario controls + telemetry).    |

The `canbridge` Dockerfile shipping a SocketCAN ↔ ROS 2 bridge is included
for offline experimentation but is **not enabled** in `docker-compose.yml` —
the validated integration uses the TCP relay above.

---

## Running the SIL

### Prerequisites

| Requirement     | Notes                                                                                |
|-----------------|--------------------------------------------------------------------------------------|
| Host OS         | Windows 11 + WSL2 Ubuntu-22.04, or native Linux                                      |
| Docker Engine   | Running **inside** WSL2 (Docker Desktop is **not** required and is not recommended). |
| Memory          | 12 GB+ allocated to WSL2 (Gazebo + ROS 2 + Zephyr build is RAM-heavy).               |
| Browser         | For dashboard (`:3000`) and noVNC view of Gazebo (`:6080`).                          |

### Quickstart

```bash
cd sil
bash ./start_sil.sh
```

Open:

- **`http://localhost:3000`** — control dashboard (CCRs/CCRm/CCRb scenarios, telemetry, FSM state)
- **`http://localhost:6080`** — noVNC view of the Gazebo 3D world
- **`http://localhost:8000/docs`** — FastAPI scenario-control endpoints

### Manual control

```bash
cd sil
sudo docker compose up --build      # build + start all services
sudo docker compose logs -f aeb-ecu # follow Zephyr ECU log
sudo docker compose down -v         # stop and remove volumes
```

### Selecting a scenario

The default scenario (`ccrs_40`) is launched on container start. To switch,
use the dashboard's scenario picker or call the API:

```bash
curl -X POST http://localhost:8000/scenario/start -d '{"name":"ccrm"}'
```

Supported scenarios — see [`aeb_gazebo/config/scenarios.yaml`](aeb_gazebo/config/scenarios.yaml):
`ccrs_20`, `ccrs_30`, `ccrs_40`, `ccrs_50`, `ccrm`, `ccrb_d2_g12`,
`ccrb_d6_g12`, `ccrb_d2_g40`, `ccrb_d6_g40`.

---

## Repository layout

```
sil/
├── README.md                  ◀── this file (top-level SIL overview)
├── docker-compose.yml         ◀── service orchestration
├── start_sil.sh               ◀── WSL2 quickstart
├── docker/                    ◀── per-service Dockerfiles + helpers
│   ├── Dockerfile.gazebo      Gazebo + ROS 2 + aeb_gazebo build
│   ├── Dockerfile.zephyr      Zephyr SDK + native_sim AEB build
│   ├── Dockerfile.api         FastAPI scenario controller
│   ├── Dockerfile.canbridge   (unused) SocketCAN ↔ ROS 2 bridge
│   ├── api/                   scenario_api.py (REST endpoints)
│   ├── canbus/                can_tcp_server.py (TCP CAN relay)
│   ├── canbridge/             bridge.py (alternative SocketCAN bridge)
│   ├── dashboard/             index.html (web UI)
│   └── nginx/                 nginx.conf
├── aeb_gazebo/                ◀── ROS 2 package (Gazebo side)
│   ├── README.md              detailed ROS 2 README (Portuguese)
│   ├── package.xml, CMakeLists.txt
│   ├── config/scenarios.yaml
│   ├── launch/*.launch.py
│   ├── models/                SDF models for ego + target vehicles
│   ├── msg/                   AEB_*.msg type definitions
│   ├── src/                   ROS 2 nodes (perception, scenario, dashboard, …)
│   └── worlds/aeb_highway.world
└── zephyr_aeb/                ◀── Zephyr application (ECU side)
    ├── CMakeLists.txt
    ├── prj.conf
    ├── boards/native_sim.overlay
    ├── patch_fsm.py           ◀── (workaround) keep brake applied below V_EGO_MIN
    └── src/
        ├── main.c             10 ms tick → aeb_core_step()
        ├── can_hal.h
        └── can_hal_zephyr.c
```

---

## Validation

A reference run against scenario `ccrs_40` is captured under
[`reports/sil/`](../reports/sil/):

- `aeb_ecu_run.log` — Zephyr ECU log, 10 ms cycles with FSM state + brake %
- `scenario_summary.csv` — distance, ego/target speed, TTC, brake, FSM state
- `fsm_transitions.txt` — recorded state transitions
- `README.md` — interpretation and acceptance criteria

**Pass criterion** (UNECE R152 / Euro NCAP CCRs): collision avoided OR
residual ego speed < 5 km/h. The reference run avoids the collision (final
distance 6.5 m, residual speed 0 km/h).

---

## CI

`.github/workflows/sil.yml` runs on PRs touching `sil/` or `src/`:

- `docker compose config` — validates the compose file
- `docker compose build` — builds every service image (smoke gate)

Full scenario execution is **not** run in CI today — it requires GUI
forwarding for Gazebo and exceeds the 6 GB / 7 GB GitHub Actions runner
budget. See *Risks / Open Points* in the related PR.

---

## Known limitations and follow-up work

1. **`patch_fsm.py` modifies `aeb_fsm.c` at build time.** This is a
   workaround for an FSM behaviour: when `v_ego < V_EGO_MIN` while in
   `BRAKE_Lx`, the FSM jumps straight to `STANDBY` and releases the brake
   before the vehicle has fully stopped. The patch keeps the brake applied
   until `v_ego < 0.01 → POST_BRAKE`. The fix should be ported into
   `src/decision/aeb_fsm.c` itself and re-validated, after which
   `patch_fsm.py` can be deleted.
2. **`Dockerfile.zephyr` clones the AEB repo from GitHub** (`main` branch)
   rather than copying the local working tree. This means PR-time SIL builds
   do not see in-PR changes to `src/`. A future iteration should switch the
   build context to the repo root and `COPY` local sources.
3. **Dashboard telemetry mixes live and demo data.** The scenario panel and
   FSM badge are live; some auxiliary panels still display placeholder
   values. The data feed from `live-data` to the dashboard JS needs the
   final wire-up.
4. **No headless scenario runner in CI.** Only the build-time smoke gate
   runs in CI. A future workflow could spawn the ECU + canbus services
   without Gazebo and replay a recorded perception trace.
