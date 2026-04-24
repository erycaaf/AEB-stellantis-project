# PID + Alert V&V — Evidence artefacts

Reproducible evidence for the independent cross-validation of
`aeb_pid.c` / `aeb_pid.h` and `aeb_alert.c` / `aeb_alert.h`.
Validator (cross): **Rian Ithalo da Costa Linhares**.
Author: **Jéssica Roberta de Souza Santos**.
Aligned to the ISO 26262-6:2018 ASIL-D technique set.

The consolidated V&V report (`Relatorio_Consolidado_VV_PID_Alert.pdf`
and its source) lives in the team documentation area, **outside this
repo**, alongside Renato's equivalent report for UDS. Only the
machine-generated evidence (logs, XML, gcov output, CSV) and the
reproducibility build targets live here.

## Reproduce everything

From the repo root, in an environment with GCC 14, gcov-14, gcovr,
cppcheck 2.13, Valgrind 3.22 and the ASan/UBSan runtimes (Ubuntu 24.04
provides all of these):

```bash
make mcdc      # Atividade 1 — structural MC/DC coverage
make fault     # Atividade 3 — systematic fault injection
make memory    # Atividade 4 — Valgrind + ASan + UBSan
make misra     # Atividade 5 — cppcheck + MISRA addon
```

This runs, in order:

1. `make mcdc`    — `-fcondition-coverage` build + gcov condition report
2. `make fault`   — 22-assertion fault-injection suite (PID + Alert)
3. `make memory`  — Valgrind + ASan + UBSan on all test binaries
4. `make misra`   — cppcheck scoped to `aeb_pid.{c,h}` and `aeb_alert.{c,h}`

## Artefact map

| File | Source activity | What it is |
|---|---|---|
| `coverage_mcdc/gcov_summary.txt` | 1 — coverage | Textual summary (stmt/branch/MC/DC %) |
| `coverage_mcdc/aeb_pid.c.gcov` | 1 — coverage | Line-by-line annotated source (PID) |
| `coverage_mcdc/aeb_alert.c.gcov` | 1 — coverage | Line-by-line annotated source (Alert) |
| `coverage_mcdc/coverage.xml` | 1 — coverage | Cobertura XML (CI-consumable) |
| `coverage_mcdc/report.html` | 1 — coverage | gcovr navigable HTML report |
| `fault_injection/run.log` | 3 — fault | 22 tests / 36 assertions, 6 failures (2 bug classes) |
| `fault_injection/pid_fault.log` | 3 — fault | PID fault-injection raw log (12/18 PASS) |
| `fault_injection/alert_fault.log` | 3 — fault | Alert fault-injection raw log (18/18 PASS) |
| `memory_safety/valgrind_test_pid.log` | 4 — memory | Valgrind, PID nominal suite (clean) |
| `memory_safety/valgrind_test_alert.log` | 4 — memory | Valgrind, Alert nominal suite (clean) |
| `memory_safety/ubsan_test_pid.log` | 4 — memory | ASan+UBSan, PID (clean) |
| `memory_safety/ubsan_test_alert.log` | 4 — memory | ASan+UBSan, Alert (clean) |
| `memory_safety/valgrind_test_*.log` | 4 — memory | Valgrind on remaining modules (all clean) |
| `memory_safety/ubsan_test_*.log` | 4 — memory | ASan+UBSan on remaining modules (all clean) |
| `misra/cppcheck_pid_alert.xml` | 5 — MISRA | cppcheck 2.13, XML (0 error/warning, 5 style — false-positive MISRA) |
| `../../functional_tests_pid_alert.csv` | traceability | Requirement → test → evidence rows (source for the shared Excel) |

## Functional Test Excel

`functional_tests_pid_alert.csv` is the canonical data for the `PID`
and `Alert` tabs of the shared Functional Test Excel (Checklist #5 of
the Final Delivery Strategy). Import into the master `.xlsx` — columns
align with the project-standard schema:

## Requirement | Test function | Input | Expected | P/F | Evidence | Category

The `Category` column is a V&V-specific extension that distinguishes
nominal tests from fault-injection categories A–D (see consolidated
report §4).

## Bugs and expected follow-ups

Six assertions failed during fault injection, concentrated in two
distinct vulnerabilities in `aeb_pid.c`. Both violate ASIL-D
fail-safe principles. Proposed patches are documented in the
consolidated V&V report.

Per ISO 26262-6 §5.4.8 (independence), the patches are to be applied
by the original author (Jéssica) on a `fix/pid-robustness` branch,
then re-validated by the cross author (Rian) running `make fault`
and `make mcdc` again.

| # | Severity | Location | Evidence (this repo) |
|---|---|---|---|
| 1 | HIGH (robustness) | `src/execution/aeb_pid.c:138` | `fault_injection/run.log` (A1, A4 — 3 assertions) |
| 2 | CRITICAL (SEU-resilience) | `src/execution/aeb_pid.c:119` | `fault_injection/run.log` (C1, C2 — 3 assertions) |

### Bug #1 — NaN propagation (HIGH)

Non-finite float inputs (`NaN`, `+Inf`, `-Inf` on `a_ego` or
`decel_target`) propagate through the PI cascade and appear at
`output->brake_pct`. The existing `clamp_f32` guard fails to detect
`NaN` because IEEE 754 comparisons against `NaN` always return false.

**Proposed patch:** add `isfinite()` guard after the existing NULL
check in `pid_brake_step` (line 102). See consolidated report §7.1.

### Bug #2 — `fsm_state` underconstrained (CRITICAL)

The guard `if (fsm_state < FSM_BRAKE_L1)` accepts any value ≥ 3
as a valid braking state. A single-event upset (SEU) flipping
`FSM_STANDBY` (1) into `129` would cause spurious braking with
undefined `decel_target`.

**Proposed patch:** replace the implicit blacklist with an explicit
whitelist of the four valid braking states
(`FSM_BRAKE_L1`..`FSM_POST_BRAKE`). See consolidated report §7.2.

## Standard alignment

| ISO 26262-6 | Item | Technique | Evidence here |
|---|---|---|---|
| Table 8 | 1b | Language subset (MISRA C:2012) | `misra/cppcheck_pid_alert.xml` |
| Table 8 | 1d | Defensive implementation | `memory_safety/*.log` |
| Table 8 | 1e | Restricted dynamic memory | `memory_safety/valgrind_*.log` (0 bytes heap) |
| Table 10 | 1b | Requirements-based testing | (tests in `tests/test_{pid,alert}.c`) |
| Table 11 | 1e | Fault injection | `fault_injection/run.log` |
| Table 12 | 1a–1c | Statement/Branch/MC/DC coverage | `coverage_mcdc/*` (100% / 100% / 100%) |
EOF

# Confere
head -30 reports/vv_pid_alert/README.md