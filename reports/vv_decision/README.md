# Decision (TTC + FSM) V&V — Evidence artefacts

Reproducible evidence for the independent cross-validation of
[src/decision/aeb_ttc.c](../../src/decision/aeb_ttc.c) and
[src/decision/aeb_fsm.c](../../src/decision/aeb_fsm.c).
**Validator (cross):** Eryca. **Author:** Lourenço Jamba Mphili.
Aligned to the ISO 26262-6:2018 ASIL-D technique set established by Rian for
PID/Alert and by Renato for UDS.

The consolidated V&V report (`Relatorio_Consolidado_VV_Decision.pdf` and its
`.tex` source) lives in the team documentation area, **outside this repo**,
alongside the equivalent reports for PID/Alert and UDS. Only the
machine-generated evidence (logs, XML, gcov output, CSV) and the
reproducibility build targets live here.

## Reproduce everything

From the repo root, in an environment with GCC 14, gcov-14, gcovr 8.6,
cppcheck 2.13, Valgrind 3.22 and the ASan/UBSan runtimes (Ubuntu 24.04
provides all of these):

```bash
make vv-decision
```

This runs (subtargets added as activities come online):

1. `make mcdc-decision` — `-fcondition-coverage` build + gcov condition report
2. *pending* — `make fault-decision` — fault-injection suite
3. *pending* — `make memory-decision` — Valgrind + ASan + UBSan
4. *pending* — `make misra-decision` — cppcheck MISRA addon scoped to decision module

## Artefact map (current)

| File | Source activity | What it is |
|---|---|---|
| `coverage_mcdc/run.log` | 1 — coverage | Test execution log (9 nominal + 36 MC/DC = 45 PASS) |
| `coverage_mcdc/coverage_summary.txt` | 1 — coverage | gcovr merged summary (aggregate across both binaries) |
| `coverage_mcdc/coverage.xml` | 1 — coverage | gcovr Cobertura XML for CI consumption |
| `coverage_mcdc/gcov_summary.txt` | 1 — coverage | gcov-14 per-binary metrics (MCDC binary only) |
| `coverage_mcdc/aeb_ttc.c.gcov` | 1 — coverage | Line-by-line annotated source (TTC) |
| `coverage_mcdc/aeb_fsm.c.gcov` | 1 — coverage | Line-by-line annotated source (FSM) |

The HTML report (`report.html` + `report.*.html` + `report.css` + `report.js`)
is regenerated locally by `make mcdc-decision` but **not committed** — browse
after running the target.

## Coverage results (aggregate, from `coverage_summary.txt`)

| Metric | Result | ASIL-D target |
|---|---|---|
| Statement (lines) | **97.7%** (173 / 177) | ≥ 95% ✅ |
| Branch | **94.4%** (117 / 124) | ≥ 95% (–0.6 pp) |
| MC/DC (conditions) | **94.0%** (109 / 116) | ≥ 95% (–1.0 pp) |
| Function | 100% (8 / 8) | — |

### Gap analysis

Four lines remain uncovered, all classified as **defensively unreachable**
(dead code per MISRA C:2012 Rule 2.1):

| File | Lines | Code | Reason unreachable |
|---|---|---|---|
| `aeb_ttc.c` | 32 | `ttc_result = 0.0f;` | `distance > 0` and `v_rel > V_REL_MIN > 0` ⇒ `ttc > 0` |
| `aeb_ttc.c` | 50 | `d_brake = 0.0f;` | `v²` is never negative in IEEE 754 (even with NaN/Inf) |
| `aeb_fsm.c` | 273–274 | `new_state = FSM_POST_BRAKE; …` | Speed guard (line 208) catches any `v_ego < V_EGO_MIN = 2.78 m/s` before this block — the inner `v_ego < 0.01` check is dominated |

These are proposed for removal in the consolidated V&V report; the patches
will be applied by the original author (Lourenço) on a separate branch per
the ISO 26262-6 §5.4.8 independence principle. Re-running `make
mcdc-decision` after the patches are merged should yield **100%** across
all three structural metrics.

## Functional Test Excel

Pending — will land as `functional_tests_decision.csv` once the full V&V
stack is complete. Schema will follow the project standard
`Requirement | Test function | Input | Expected | P/F | Evidence`, with
rows for FR-DEC-001 to FR-DEC-011 and FR-FSM-001 to FR-FSM-006.

## Bugs and expected follow-ups

No robustness bugs identified yet (activities 3 and 4 — fault injection and
memory safety — still pending). Structural findings above will be logged as
**dead-code removal suggestions**, not behavioural defects.
