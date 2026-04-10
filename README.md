# **Autonomous Emergency Braking (AEB)**  

> Technological Residency in Embedded Software Development for the Automotive Sector  
> Federal University of Pernambuco (UFPE) · Center for Informatics · Stellantis

![MISRA C](https://img.shields.io/badge/MISRA%20C-2012-blue)
![ISO 26262](https://img.shields.io/badge/ISO-26262-orange)
![UNECE R152](https://img.shields.io/badge/UNECE-R152-green)
![ISO 14229](https://img.shields.io/badge/ISO-14229-lightgrey)
![Status](https://img.shields.io/badge/Status-Academic%20Project-informational)

---

## Overview

This repository contains the requirements baseline, model-based design artefacts, software implementation assets, and configuration-management workflow for an **Autonomous Emergency Braking (AEB)** system focused on **rear-end collision mitigation**.

The system is designed to detect imminent rear-end collision risk with a vehicle ahead, warn the driver, and autonomously apply progressive braking when required to avoid or mitigate impact severity.

The current project baseline is limited to **longitudinal control only**. No lateral collision-avoidance manoeuvres, such as steering interventions, are included.

---

## Project Scope

The AEB baseline documented in this repository is defined by the following characteristics:

- rear-end collision mitigation against a vehicle ahead
- operation in the **10–60 km/h** ego-vehicle speed range
- dual decision criterion based on **Time-To-Collision (TTC)** and **minimum braking distance**
- progressive intervention through a **7-state Finite State Machine**
- communication support through **CAN**
- diagnostic support through **UDS**
- traceability from **requirements → model → C implementation → Tests**

### In Scope

- **CCRs** — Car-to-Car Rear Stationary
- **CCRm** — Car-to-Car Rear Moving
- **CCRb** — Car-to-Car Rear Braking
- requirements engineering and traceability
- MIL-oriented model validation
- embedded C implementation aligned with the validated design baseline
- software quality constraints derived from MISRA C and ISO 26262-oriented practices

### Out of Scope

- vulnerable road users (pedestrians, cyclists)
- lateral collision avoidance
- production ECU electrical integration
- real sensor-fusion deployment
- adverse weather and complex road geometries beyond straight longitudinal scenarios

---
## Operational States

The validated AEB state machine contains seven states:

1. **OFF**
2. **STANDBY**
3. **WARNING**
4. **BRAKE_L1**
5. **BRAKE_L2**
6. **BRAKE_L3**
7. **POST_BRAKE**

```mermaid
flowchart LR
    OFF(["OFF"])
    STANDBY(["STANDBY"])
    WARNING(["WARNING<br/>visual + audible alert"])
    L1(["BRAKE_L1<br/>−2 m/s²"])
    L2(["BRAKE_L2<br/>−4 m/s²"])
    L3(["BRAKE_L3<br/>−6 m/s²"])
    PB(["POST_BRAKE<br/>−6 m/s² · 2 s"])

    OFF -->|"clean system status"| STANDBY
    STANDBY -->|"sensor fault"| OFF
    STANDBY -->|"TTC ≤ 4.0 s"| WARNING
    WARNING -->|"TTC > 4.0 s for 200 ms"| STANDBY
    WARNING -->|"TTC ≤ 3.0 s + 0.8 s warning"| L1
    WARNING -->|"TTC ≤ 2.2 s + 0.8 s warning"| L2
    WARNING -->|"TTC ≤ 1.8 s + 0.8 s warning"| L3
    L1 -->|"TTC ≤ 2.2 s"| L2
    L1 -->|"TTC ≤ 1.8 s"| L3
    L1 -->|"TTC > 3.0 s for 200 ms"| WARNING
    L1 -->|"vehicle speed = 0"| PB
    L2 -->|"TTC ≤ 1.8 s"| L3
    L2 -->|"TTC > 2.2 s for 200 ms"| L1
    L2 -->|"vehicle speed = 0"| PB
    L3 -->|"TTC > 1.8 s for 200 ms"| L2
    L3 -->|"vehicle speed = 0"| PB
    PB -->|"2.0 s elapsed"| STANDBY
```

**Floor distance** (prevent premature de-climbing while braking):
- d ≤ 20 m → maintain minimum BRAKE_L1
- d ≤ 10 m → maintain minimum BRAKE_L2
- d ≤ 5 m → maintain minimum BRAKE_L3

---

### Intervention semantics

- **WARNING** provides prior driver notification
- **BRAKE_L1 / L2 / L3** represent progressively stronger autonomous braking
- **POST_BRAKE** maintains brake hold after vehicle stop and manages post-stop release

---

## Validation Scenarios (Euro NCAP CCR)

The requirements baseline defines representative validation scenarios and acceptance criteria:

| Scenario | Condition | Acceptance Criterion |
|---|---|---|
| **CCRs** | Ego vehicle at 40 km/h, stationary target | Complete stop or residual speed < 5 km/h |
| **CCRm** | Ego vehicle at 50 km/h, target at 20 km/h | Collision avoided or impact speed reduced by at least 20 km/h |
| **CCRb** | Ego vehicle at 50 km/h, target decelerating at −2 m/s² | Collision avoided or impact speed < 15 km/h |

In all validated scenarios, the project monitors final distance, residual speed, braking behaviour, and state transitions.

---

## Architecture

The repository follows the classic **three-layer ADAS architecture**, in which perception provides validated input data, decision logic evaluates collision risk, and execution applies alerts and braking actions. In addition, **CAN** and **UDS** support communication and diagnostic functions across the system.

| Block | Input | Processing | Output |
|---|---|---|---|
| **Perception** | Distance, ego speed, target speed, fault indicators | Validation, plausibility checks, signal conditioning | Trusted signals |
| **Decision** | Trusted signals, override inputs | TTC, braking-floor logic, FSM transitions, risk assessment | Alert/braking request |
| **Execution** | Alert/braking request | Alert generation and brake control | Brake command, alert output |
| **CAN** | System signals | Communication transport and timeout handling | Encoded/decoded messages |
| **UDS** | Diagnostic requests | Data access, DTC handling, enable/disable routines | Diagnostic responses |
---

## Software Decomposition

The requirements-to-software traceability baseline maps the main functional domains to the following implementation modules:

| Module | Responsibility |
|---|---|
| `aeb_perception.c` | sensor acquisition and validation |
| `aeb_ttc.c` | TTC and braking-distance calculation |
| `aeb_fsm.c` | risk classification, distance floors, and state-machine control |
| `aeb_alert.c` | visual and audible alert outputs |
| `aeb_pid.c` | braking control and brake-command generation |
| `aeb_can.c` | CAN encoding, decoding, and timeout handling |
| `aeb_uds.c` | UDS diagnostic services |

This modular structure supports portability, maintainability, and bidirectional traceability.

---
## Compliance and Standards

| Aspect | Standard | Status |
|---|---|---|
| Embedded C code | MISRA C:2012 | ✅ Implemented |
| Functional safety | ISO 26262 ASIL-B | ✅ Compliant architecture |
| Test scenarios | Euro NCAP AEB CCR v4.3 | ✅ CCRs, CCRm, CCRb |
| AEB protocol | UNECE R152 | ✅ Alert ≥ 0.8 s before braking |
| CAN bus | Proprietary DBC file | ✅ 5 structured frames |
| Sensor fusion | ISO 15622 | ✅ Weighted radar + lidar |

---

## AI-Assisted Development

This project used Large Language Model (LLM) assistants as a development support tool during the embedded C implementation phase. The use of AI was guided by the team's implementation strategy document and followed a structured prompt methodology to ensure consistency, traceability, and compliance with project standards.

### Principles

All AI-generated code was treated as a **draft that requires human validation**. The following principles were applied throughout the project:

1. **Human accountability** — Every team member remains fully responsible for the code submitted under their name. AI output was reviewed, tested, and modified before integration.
2. **Specification-driven prompts** — Prompts were constructed from authoritative project artefacts: the SRS v2.0, the Simulink model (AEB_Integration.slx), the interface contract (`aeb_types.h`, `aeb_config.h`), and the applicable MISRA C:2012 rules.
3. **Verification before commit** — All AI-generated code was compiled with strict warnings (`-Wall -Wextra -Wpedantic -std=c99`), validated against unit tests, and checked for MISRA compliance before being committed to the repository.
4. **Traceability preserved** — AI-generated code follows the same requirement-to-code traceability as manually written code. Every function includes `@requirement` tags linking to SRS identifiers.

### Tools Used

| Tool | Provider | Usage |
|---|---|---|
| Claude (Opus / Sonnet) | Anthropic | Code generation, test creation, design documentation, code review preparation |
| DeepSeek | DeepSeek AI | Code generation support |

### Scope of AI Assistance

| Activity | AI Role | Human Role |
|---|---|---|
| C module implementation | Generate initial code from structured prompts containing interface contracts, Simulink logic, and requirements | Review for correctness, MISRA compliance, and architectural consistency |
| Unit test creation | Generate test cases covering requirement acceptance criteria and boundary conditions | Validate test logic, verify assertion correctness, add domain-specific scenarios |
| Code review preparation | Draft PR descriptions and design rationale | Review and adjust content for accuracy |
| MISRA compliance fixes | Suggest restructuring for single exit point, const qualifiers, NULL guards | Verify fix correctness and re-run static analysis |

---

## Authors
- Eryca Francyele de Moura e Silva
- Jéssica Roberta de Souza Santos
- Lourenço Jamba Mphili
- Renato Silva Fagundes
- Rian Ithalo da Costa Linhares
