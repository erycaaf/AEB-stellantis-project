# AEB SIL — Software-in-the-Loop Validation

End-to-end Software-in-the-Loop environment for the AEB system. Runs the
embedded C code (`src/`) inside a Zephyr `native_sim` ECU and exercises it
against a Gazebo Classic 11 vehicle simulation, with a TCP CAN bus relaying
frames between the perception, ECU, and scenario containers.

> **Status:** integrated and validated — CCRs at 40 km/h stops at ~6 m,
> collision avoided, FSM sequence
> `STANDBY → WARNING → BRAKE_L1 → BRAKE_L2 → POST_BRAKE → STANDBY`.
> See [`reports/sil/`](../reports/sil/) for run evidence.

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
        │ ─ rural world    │                   │  ─ aeb_can · aeb_uds         │
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
| `gazebo`    | `osrf/ros:humble-desktop` (built)    | Gazebo 11 + ROS 2 Humble + the `aeb_gazebo` package (perception, scenario, world). Bundles a curated subset of `osrf/gazebo_models` (oak/pine trees, mailbox, fire hydrant, houses, gas station, parked vehicles, cow). |
| `aeb-ecu`   | `ghcr.io/zephyrproject-rtos/ci`      | Zephyr `native_sim` running `aeb_core` end-to-end on a 10 ms cycle.            |
| `api`       | `osrf/ros:humble-desktop` (built)    | FastAPI scenario-control endpoints (`:8000`) used by the dashboard.            |
| `dashboard` | `nginx:alpine`                       | Web UI at `:3000` — embedded Gazebo view (noVNC) + scenario controls + telemetry. |

The `canbridge` Dockerfile shipping a SocketCAN ↔ ROS 2 bridge is included
for offline experimentation but is **not enabled** in `docker-compose.yml` —
the validated integration uses the TCP relay above.

---

## Running the SIL

### Prerequisites

| Requirement     | Notes                                                                                |
|-----------------|--------------------------------------------------------------------------------------|
| Host OS         | Windows 10/11 + WSL2 Ubuntu-22.04, or native Linux                                   |
| Docker          | Either Docker Engine inside WSL2 **or** Docker Desktop on Windows (both work — the stack uses TCP CAN only, so no kernel/SocketCAN dependency). |
| Memory          | ~6 GB free for the runtime stack (Gazebo + ROS 2 nodes + Zephyr ECU). Cold builds peak around 2–3 GB. WSL2 default of 50 % of host RAM is usually enough on a 16 GB+ host. |
| Disk            | ~8 GB free for image layers (osrf/ros + Zephyr SDK + cached gazebo_models clone).    |
| Browser         | For the dashboard at `http://localhost:3000`.                                        |
| First-run time  | ~12–15 min for Docker images to build. Cached rebuilds finish in seconds.            |

### Quickstart

```bash
cd sil
bash ./start_sil.sh
```

The script auto-detects whether Docker is already running (Docker Desktop or
an existing Engine in WSL2) and only starts `dockerd` itself when neither is
available. It then runs `docker compose up --build`.

### Manual control

```bash
cd sil
docker compose up --build      # build + start all services (no sudo on Docker Desktop)
docker compose logs -f aeb-ecu # follow Zephyr ECU log
docker compose down -v         # stop and remove volumes
```

(Prefix with `sudo` if you're using Docker Engine in WSL2 without your user
in the `docker` group.)

### Web UI

| URL                              | What it shows                                                            |
|----------------------------------|--------------------------------------------------------------------------|
| `http://localhost:3000`          | Main dashboard — scenario picker, Start / Stop / Reset, telemetry, embedded Gazebo 3D view. |
| `http://localhost:6080/vnc.html` | Direct noVNC view of the Gazebo 3D scene (the same stream the dashboard embeds). |
| `http://localhost:8000/docs`     | FastAPI Swagger UI — REST endpoints for scenario control.                |

### Selecting a scenario

The `ccrs_40` scenario auto-loads on container start. Switch via the
dashboard's dropdown + **Start**, or via the API:

```bash
curl -X POST http://localhost:8000/scenarios/start \
     -H "Content-Type: application/json" \
     -d '{"scenario":"ccrm"}'
```

| Scenario       | Ego (km/h) | Target (km/h) | Gap (m) | Pass criterion              |
|----------------|-----------:|--------------:|--------:|-----------------------------|
| `ccrs_20/30/40/50` | 20–50  |             0 |     100 | residual `v_impact < 5 km/h`|
| `ccrm`         |         50 |            20 |      50 | `Δv ≥ 20 km/h`              |
| `ccrb_d2_g12`  |         50 |    50 → −2 m/s² |      12 | `v_impact < 15 km/h`        |
| `ccrb_d6_g12`  |         50 |    50 → −6 m/s² |      12 | `v_impact < 15 km/h`        |
| `ccrb_d2_g40`  |         50 |    50 → −2 m/s² |      40 | `v_impact < 15 km/h`        |
| `ccrb_d6_g40`  |         50 |    50 → −6 m/s² |      40 | `v_impact < 15 km/h`        |

Definitive list lives in [`aeb_gazebo/config/scenarios.yaml`](aeb_gazebo/config/scenarios.yaml).
The same dict is mirrored in `aeb_gazebo/src/scenario_controller.py` and the
launch file — keep them in sync.

---

## Setup on a fresh Windows + WSL machine

The most common gotchas a new contributor hits:

### 1. Docker Engine vs Docker Desktop

Both work. Docker Desktop is the easier path on Windows — install it,
enable the WSL2 integration for your distro, and the `docker` CLI works
inside the Ubuntu shell. If you prefer Docker Engine inside WSL2 (lighter,
no subscription requirements), follow the [official engine instructions](https://docs.docker.com/engine/install/ubuntu/)
and add yourself to the `docker` group.

The `start_sil.sh` script handles either by detecting `docker info` first
and only invoking `sudo dockerd` when no daemon is running.

### 2. Subnet conflicts

`docker-compose.yml` pins the project network to `10.99.0.0/24` (canbus at
`10.99.0.10`) so the Zephyr ECU's CAN HAL can connect via a literal IP —
the Zephyr POSIX subset on `native_sim` doesn't expose `getaddrinfo`, so it
can't resolve service names.

If your machine already has `10.99.0.0/24` allocated to another Docker
network, `docker compose up` fails with *"Pool overlaps with other one on
this address space"*. Fix by editing [`docker-compose.yml`](docker-compose.yml):
swap `10.99.0.0/24` → an unused private range, and the matching
`ipv4_address: 10.99.0.10` and `CAN_TCP_HOST=10.99.0.10` env values.

You can list existing Docker networks with `docker network ls --no-trunc`.

### 3. WSL2 memory

If a cold build OOMs (gets killed mid-`west update` or mid-`colcon build`),
bump WSL2's memory ceiling in `%UserProfile%\.wslconfig` (Windows side):

```ini
[wsl2]
memory=8GB
processors=4
```

then `wsl --shutdown` and reopen the Ubuntu shell.

### 4. Port conflicts

The stack binds host ports `3000` (dashboard), `6080` (noVNC), `8000`
(API), and `29536` (TCP CAN relay). If anything else is listening on those,
edit the `ports:` mappings in `docker-compose.yml`.

### 5. Stale `gzclient` lock

If `gzclient` ever fails to render after a previous run, the gazebo
container's bash entrypoint already cleans `/tmp/.X*-lock` and
`/tmp/.X11-unix/` on boot. A `docker compose down -v` is the nuclear
option that also clears the persistent `live-data` volume.

---

## Repository layout

```
sil/
├── README.md                  ◀── this file (top-level SIL overview)
├── docker-compose.yml         ◀── service orchestration
├── start_sil.sh               ◀── WSL2 quickstart (Docker Desktop friendly)
├── docker/                    ◀── per-service Dockerfiles + helpers
│   ├── Dockerfile.gazebo      Gazebo + ROS 2 + aeb_gazebo + gazebo_models bundle
│   ├── Dockerfile.zephyr      Zephyr SDK + native_sim AEB build (applies patch_fsm.py)
│   ├── Dockerfile.api         FastAPI scenario controller
│   ├── Dockerfile.canbridge   (unused) SocketCAN ↔ ROS 2 bridge
│   ├── api/                   scenario_api.py (REST endpoints)
│   ├── canbus/                can_tcp_server.py (TCP CAN relay)
│   ├── canbridge/             bridge.py (alternative SocketCAN bridge)
│   ├── dashboard/             index.html (web UI — circular speedometer, FSM pills,
│   │                          TTC bar, brake gauge, alerts, embedded Gazebo iframe)
│   └── nginx/                 nginx.conf (reverse-proxies /api/ and /novnc/)
├── aeb_gazebo/                ◀── ROS 2 package (Gazebo side)
│   ├── README.md              detailed ROS 2 README (Portuguese)
│   ├── package.xml, CMakeLists.txt
│   ├── config/scenarios.yaml  preset definitions (ego speed, target speed, gap, …)
│   ├── launch/*.launch.py     scenario launchers
│   ├── models/                SDF wrapper models for ego + target vehicles
│   ├── msg/                   AEB_*.msg type definitions
│   ├── src/                   ROS 2 nodes:
│   │                            scenario_controller (drives ego/target, end criteria)
│   │                            perception_node     (radar + lidar fusion → CAN)
│   │                            dashboard_node      (legacy matplotlib panel)
│   │                            can_tcp_client      (Python TCP CAN client)
│   └── worlds/aeb_highway.world  3-lane road + rural scenery (houses, gas station,
│                                  trees, cow, guardrail, dirt drives, distance markers)
└── zephyr_aeb/                ◀── Zephyr application (ECU side)
    ├── CMakeLists.txt
    ├── prj.conf
    ├── boards/native_sim.overlay
    ├── patch_fsm.py           ◀── (workaround) early POST_BRAKE entry below V_EGO_MIN
    └── src/
        ├── main.c             10 ms tick → aeb_core_step()
        ├── can_hal.h
        └── can_hal_zephyr.c   POSIX-socket TCP CAN client (uses inet_pton — no DNS)
```

---

## Validation

A reference run against scenario `ccrs_40` is captured under
[`reports/sil/`](../reports/sil/):

- `aeb_ecu_run.log` — Zephyr ECU log, 10 ms cycles with FSM state + brake %
- `gazebo_scenario.log` — scenario_controller + perception_node output
- `canbus_frames.log` — TCP CAN relay frame counter
- `services.txt` — `docker compose ps` snapshot
- `README.md` — interpretation and acceptance criteria

**Pass criterion** (UNECE R152 / Euro NCAP CCRs): collision avoided OR
residual ego speed `< 5 km/h`. The reference run avoids the collision (final
distance ~6 m, residual speed 0 km/h, FSM cascade traverses every state).

---

## CI

[`.github/workflows/sil.yml`](../.github/workflows/sil.yml) runs on PRs
touching `sil/`, `src/`, or `include/`:

- `docker compose config` — validates the compose file
- `hadolint` — lint Dockerfiles
- `docker build` for the small images (`api`, `canbridge`)
- `python -m compileall` + `ruff check` on all SIL Python sources
- `shellcheck` on `start_sil.sh`, `aeb_gazebo/run.sh`, `aeb_gazebo/launcher.sh`

Heavy images (`gazebo`, `aeb-ecu`) are **not** built in CI — they exceed
the GitHub-hosted runner's RAM/disk budget. Full scenario execution is also
not run in CI today (needs GUI + Gazebo). See *Risks / Open Points* in the
related PR for the path forward.

---

## Known limitations and follow-up work

1. **`patch_fsm.py` modifies `aeb_fsm.c` at build time** (tracks
   [#95](https://github.com/erycaaf/AEB-stellantis-project/issues/95)).
   The patch makes the FSM enter `POST_BRAKE` as soon as the ego drops
   below `V_EGO_MIN` while braking, instead of forcing `STANDBY` and
   releasing the brakes. The proper fix is to re-shape the Priority 3
   speed-range branch in `src/decision/aeb_fsm.c` so the case "actively
   braking, speed has dipped below `V_EGO_MIN`" no longer falls into the
   `else { FSM_STANDBY; }` clear-everything block — see
   [`zephyr_aeb/patch_fsm.py`](zephyr_aeb/patch_fsm.py) for the exact
   replacement. Once that lands and is re-validated, delete `patch_fsm.py`
   and the `RUN python3 /app/zephyr_aeb/patch_fsm.py …` step in
   `Dockerfile.zephyr`.

2. **Coast-decel hack in `scenario_controller.py`.** Gazebo's
   `gazebo_ros_planar_move` plugin has no friction model, so an ego with
   non-zero `cmd_vel` rolls forever. After the AEB releases the brake we
   manually decelerate at 2 m/s² until the ego matches the target's speed
   (zero for CCRs, target's speed for CCRm/CCRb). Removing this requires
   either adding wheel/contact friction to the world or moving the
   simulated brake hold into a follow-up POST_BRAKE phase in the AEB code
   itself.

3. **`Dockerfile.zephyr` clones the AEB repo from GitHub** (`development`
   branch — that branch carries the latest robustness work and is ahead
   of `main`). This still means PR-time SIL builds do not see in-PR
   changes to `src/`. A future iteration should switch the build context
   to the repo root and `COPY` local sources so a PR's own modifications
   are exercised by its own SIL run.

4. **Zephyr CAN HAL doesn't resolve hostnames.** `can_hal_zephyr.c` uses
   `inet_pton`, so `CAN_TCP_HOST` must be a dotted-IPv4 literal — that's
   why the compose file pins a static IP for the canbus service. If
   Zephyr's POSIX subset gains `getaddrinfo`, the static-IP requirement
   can go away.

5. **`/reset_world` doesn't fully zero the planar_move plugin's odom
   integrator.** As a result the controller now teleports both vehicles
   itself via `set_entity_state` on every restart, with their `Twist`
   zeroed. If a future Gazebo Classic version fixes this we can drop the
   manual teleport.

6. **No headless scenario runner in CI.** Only the build-time smoke gate
   runs in CI. A future workflow could spawn the ECU + canbus services
   without Gazebo and replay a recorded perception trace, comparing FSM
   transitions against a golden log.
