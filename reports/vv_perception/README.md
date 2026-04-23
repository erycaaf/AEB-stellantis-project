# Perception V&V — Evidence artefacts

Reproducible evidence for the independent cross-validation of `aeb_perception.c` /
`aeb_perception.h`. Validator (cross): Jéssica. Author: Eryca.
Aligned to the ISO 26262-6:2018 ASIL-D technique set previously applied by Renato
to the UDS module.

The consolidated V&V report (`Relatorio_Consolidado_VV_Perception.pdf` and its
`.tex` source) lives in the team documentation area, **outside this repo**,
alongside the equivalent reports for UDS and PID/Alert. Only the
machine-generated evidence (logs, XML, gcov output, CSV) and the reproducibility
build targets live here.

## Reproduce everything

From the repo root, in an environment with GCC 14, gcov-14, gcovr, cppcheck 2.13,
Valgrind 3.22 and the ASan/UBSan runtimes (Ubuntu 24.04 provides all of these —
see `docker/Dockerfile.vv` for the containerised setup; same image used for UDS):

```bash
make vv-perception
```

This runs, in order:

1. `make mcdc-perception` — `-fcondition-coverage` build + gcov condition report
2. `make fault-perception` — 25-assertion fault-injection suite
3. `make memory-perception` — Valgrind + ASan + UBSan on both nominal and fault binaries
4. `make misra-perception` — cppcheck MISRA addon scoped to `aeb_perception.{c,h}`

## Artefact map

| File | Source activity | What it is |
|---|---|---|
| `coverage_mcdc/run.log`            | 1 — coverage | nominal suite execution log (25/25 PASS) |
| `coverage_mcdc/gcov_summary.txt`   | 1 — coverage | gcov text summary (stmt/branch/MC/DC %) |
| `coverage_mcdc/aeb_perception.c.gcov` | 1 — coverage | line-by-line annotated source |
| `fault_injection/run.log`          | 3 — fault    | 25 assertions, 9 failures (3 bug classes) |
| `memory_safety/valgrind_test_perception.log`        | 4 — memory | Valgrind, nominal (clean) |
| `memory_safety/valgrind_test_perception_fault.log`  | 4 — memory | Valgrind, fault suite (clean) |
| `memory_safety/ubsan_test_perception.log`           | 4 — memory | UBSan, nominal (clean) |
| `memory_safety/ubsan_test_perception_fault.log`     | 4 — memory | UBSan, fault suite (clean — see report §4) |
| `misra/cppcheck_perception.xml`    | 5 — MISRA    | cppcheck 2.13 + MISRA addon, XML (3 advisory issues, Rule 15.5) |
| `functional_tests_perception.csv`  | traceability | Requirement → test → evidence rows (source for the shared Excel) |

## Functional Test Excel

`functional_tests_perception.csv` is the canonical data for the `Perception` tab of the
shared Functional Test Excel (Checklist #5 of the Final Delivery Strategy). Import into
the master `.xlsx` as the `Perception` tab — columns align with the project-standard
schema `Requirement | Test function | Input | Expected | P/F | Evidence`.

The `Category` column is a V&V-specific extension that distinguishes nominal tests from
fault-injection categories A–D (see report §4).

## Key result vs. UDS module

Unlike UDS, which reached 100% MC/DC with the nominal suite alone, the Perception
module's nominal suite reaches only **88.04% MC/DC** (81/92 condition outcomes). The
gap analysis is in report §3.3 with a fix plan (promote two fault-injection cases to
the nominal suite + deviation analysis for the rest).

Also unlike UDS, **UBSan does not catch any of Perception's bugs** — Perception emits
`float32_t` (no float→int cast), so the NaN propagation defect flows through perfectly
legal IEEE-754 arithmetic and is invisible to the sanitiser. **Fault injection is the
only activity that surfaces these bugs**, which strengthens the case for it as a
non-substitutable ASIL-D technique.

## Bugs and expected follow-ups

Three vulnerabilities were identified in the fault-injection activity; proposed
patches are documented in the consolidated V&V report. Per ISO 26262-6 §5.4.8
(independence), the patches are to be applied by the original author (Eryca)
on a `fix/perception-robustness` branch, then re-validated by the cross author
(Jéssica) running `make vv-perception` again.

| # | Severity | Location | Failing assertions | Evidence (this repo) |
|---|---|---|---|---|
| 1 | HIGH (robustness) | `src/perception/aeb_perception.c:41`, `:86-87` | FAULT-A1, A2, A3, A4 (7 assertions) | `fault_injection/run.log` |
| 2 | MEDIUM (robustness) | `src/perception/aeb_perception.c:286` | FAULT-A5 (1 assertion) | `fault_injection/run.log` |
| 3 | MEDIUM (false-positive latch) | `src/perception/aeb_perception.c:55`, `:102` | FAULT-D2 (1 assertion) | `fault_injection/run.log` |
