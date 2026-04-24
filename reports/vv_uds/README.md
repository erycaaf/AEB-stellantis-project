# UDS V&V — Evidence artefacts

Reproducible evidence for the independent cross-validation of `aeb_uds.c` /
`aeb_uds.h`. Validator (cross): Renato Fagundes. Author: Rian Linhares.
Aligned to the ISO 26262-6:2018 ASIL-D technique set that Rian applied to
PID/Alert.

The consolidated V&V report (`Relatorio_Consolidado_VV_UDS.pdf` and its `.tex`
source) lives in the team documentation area, **outside this repo**, alongside
Rian's equivalent report for PID/Alert. Only the machine-generated evidence
(logs, XML, gcov output, CSV) and the reproducibility build targets live here.

## Reproduce everything

From the repo root, in an environment with GCC 14, gcov-14, gcovr, cppcheck 2.13,
Valgrind 3.22 and the ASan/UBSan runtimes (Ubuntu 24.04 provides all of these —
see `docker/Dockerfile.vv` for a containerised setup):

```bash
make vv-uds
```

This runs, in order:

1. `make mcdc-uds` — `-fcondition-coverage` build + gcov condition report
2. `make fault-uds` — 18-assertion fault-injection suite
3. `make memory-uds` — Valgrind + ASan + UBSan on both nominal and fault binaries
4. `make misra-uds` — cppcheck MISRA addon scoped to `aeb_uds.{c,h}`

## Artefact map

| File | Source activity | What it is |
|---|---|---|
| `coverage_mcdc/run.log`        | 1 — coverage | nominal suite execution log (23/23 PASS) |
| `coverage_mcdc/gcov_summary.txt` | 1 — coverage | gcov text summary (stmt/branch/MC/DC %) |
| `coverage_mcdc/aeb_uds.c.gcov` | 1 — coverage | line-by-line annotated source |
| `fault_injection/run.log`      | 3 — fault    | 18 assertions, 7 failures (3 bug classes) |
| `memory_safety/valgrind_test_uds.log`         | 4 — memory | Valgrind, nominal (clean) |
| `memory_safety/valgrind_test_uds_fault.log`   | 4 — memory | Valgrind, fault suite (clean) |
| `memory_safety/ubsan_test_uds.log`            | 4 — memory | UBSan, nominal (clean) |
| `memory_safety/ubsan_test_uds_fault.log`      | 4 — memory | UBSan, fault suite (2 UB sites: aeb_uds.c:82, :103) |
| `misra/cppcheck_uds.xml`       | 5 — MISRA    | cppcheck 2.13 + MISRA addon, XML (0 issues in UDS scope) |
| `functional_tests_uds.csv`     | traceability | Requirement → test → evidence rows (source for the shared Excel) |

## Functional Test Excel

`functional_tests_uds.csv` is the canonical data for the `UDS` tab of the shared
Functional Test Excel (Checklist #5 of the Final Delivery Strategy). Import into
the master `.xlsx` as the `UDS` tab — columns align with the project-standard
schema `Requirement | Test function | Input | Expected | P/F | Evidence`.

The `Category` column is a V&V-specific extension that distinguishes nominal
tests from fault-injection categories A–D (see report §4).

## Bugs and expected follow-ups

Three vulnerabilities were identified in the fault-injection activity; proposed
patches are documented in the consolidated V&V report. Per ISO 26262-6 §5.4.8
(independence), the patches are to be applied by the original author (Rian)
on a `fix/uds-robustness` branch, then re-validated by the cross author
(Renato) running `make vv-uds` again.

| # | Severity | Location | Evidence (this repo) |
|---|---|---|---|
| 1 | HIGH (robustness) | `src/communication/aeb_uds.c:82`, `:103` | `fault_injection/run.log` (B1/B2/B4/B5) + `memory_safety/ubsan_test_uds_fault.log` |
| 2 | CRITICAL (SEU) | `src/communication/aeb_uds.c:275` | `fault_injection/run.log` (C1) |
| 3 | CRITICAL (SEU) | `src/communication/aeb_uds.c:277–287` | `fault_injection/run.log` (C2, C3) |
