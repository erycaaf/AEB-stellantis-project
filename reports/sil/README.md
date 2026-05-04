# SIL Validation Runs

Captured Euro NCAP scenario runs from the Docker SIL stack
(`sil/docker-compose.yml`). Each `runs/<scenario>.log` contains:

- `scenario_controller` telemetry rows (every second): time, distance,
  ego speed, commanded brake %, FSM state.
- The last ~20 cycles of the Zephyr ECU log (`aeb-ecu`) showing the
  end-state — typically `st=6` (POST_BRAKE) or `st=1` (STANDBY) with
  `flt=0` after a clean stop.

These were collected with the dashboard / `/scenarios/start` API in a
back-to-back sweep — the api restarts the ECU container at the start of
each scenario, which is what makes the multi-run sweep clean (the
Zephyr `aeb_perception.c` ROC fault watchdog otherwise latches across
runs after the first scenario teleport jumps `prev_d` by > 10 m).

## Result summary

Scenario `pass criterion` from `sil/aeb_gazebo/config/scenarios.yaml`.

| Scenario       | Pass criterion        | Final clearance | v_impact  | FSM cascade observed                                    | Verdict |
|----------------|-----------------------|-----------------|-----------|----------------------------------------------------------|---------|
| `ccrs_20`      | `v_impact < 5 km/h`   | 10.1 m          | 0 km/h    | `STANDBY → WARNING → BRAKE_L1 → BRAKE_L2 → POST_BRAKE`   | **PASS** |
| `ccrs_30`      | `v_impact < 5 km/h`   |  8.2 m          | 0 km/h    | `STANDBY → WARNING → BRAKE_L1 → BRAKE_L2 → POST_BRAKE`   | **PASS** |
| `ccrs_40`      | `v_impact < 5 km/h`   |  5.3 m          | 0 km/h    | `STANDBY → WARNING → BRAKE_L1 → BRAKE_L2 → POST_BRAKE`   | **PASS** |
| `ccrs_50`      | `v_impact < 5 km/h`   |  4.7 m          | 0 km/h    | `STANDBY → WARNING → BRAKE_L1 → BRAKE_L2 → POST_BRAKE`   | **PASS** |
| `ccrm`         | no collision          | target moves away (target_v > ego_v after AEB) | 0 km/h | `STANDBY → BRAKE_L2 → STANDBY` (AEB releases once threat resolved) | **PASS** |
| `ccrb_d2_g40`  | `v_impact < 5 km/h`   |  1.4 m          | 0 km/h    | `BRAKE_L1 → BRAKE_L2 → BRAKE_L3 → POST_BRAKE`            | **PASS** |
| `ccrb_d6_g40`  | `v_impact < 5 km/h`   | 0 m (collision) | 19.7 km/h | `BRAKE_L3` (engaged late, ran out of stopping distance)  | **fail** |

`ccrb_d6_g40` (target braking at 6 m/s² in 40 m) is the hardest Euro
NCAP profile — even production AEBs commonly cannot stop in time. AEB
*did* engage at L3 / 60 % brake; the failure is FSM tuning territory
(threshold timing / TTC margins), not a SIL bug.

## How to reproduce

```bash
cd sil
sudo docker compose up --build -d
# Wait ~30 s for gazebo healthcheck.

for s in ccrs_20 ccrs_30 ccrs_40 ccrs_50 ccrm ccrb_d2_g40 ccrb_d6_g40; do
  curl -s -X POST http://localhost:8000/scenarios/start \
    -H "Content-Type: application/json" \
    -d "{\"scenario\":\"$s\"}" > /dev/null
  sleep 22
  sudo docker compose logs --since 22s gazebo 2>&1 \
    | grep -E "scenario_controller.*(t=|COLLISION|STOPPED|>>> Scenario)" \
    | sed 's/\x1b\[[0-9;]*m//g' \
    > "../reports/sil/runs/${s}.log"
  sudo docker compose logs --since 22s aeb-ecu 2>&1 \
    | tail -25 | sed 's/\x1b\[[0-9;]*m//g' \
    >> "../reports/sil/runs/${s}.log"
done
```

The dashboard at `http://localhost:3000` produces the same flow with a
clickable UI.
