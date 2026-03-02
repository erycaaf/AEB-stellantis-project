# 🚗 Autonomous Emergency Braking (AEB) System
![CI Build](https://img.shields.io/badge/build-passing-brightgreen) ![Coverage](https://img.shields.io/badge/coverage-100%25-brightgreen) ![MISRA-C](https://img.shields.io/badge/compliance-MISRA--C-blue)

**CIn UFPE & Stellantis Technological Residence - Final Project**

## 📌 Project Overview
This repository contains the full embedded software implementation of an Autonomous Emergency Braking (AEB) system. The project strictly follows the automotive V-Model lifecycle, covering requirements engineering, Simulink modeling, C-code implementation, and 3D simulation.

The system is designed to detect rear-end collision risks (CCRs, CCRm, CCRb scenarios) and autonomously apply braking interventions to avoid or mitigate impacts, adhering to **Euro NCAP** and **UNECE R152** standards.

## 🛠️ Technology Stack & Tools
*   **Core Implementation:** C (MISRA C:2012 compliant)
*   **Modeling & Control:** MATLAB / Simulink / Stateflow
*   **Testing & Validation:** Google Test (C++), gcov (Coverage)
*   **Simulation Environment:** Gazebo 3D / ROS 2
*   **CI/CD Pipeline:** GitHub Actions (Automated build, test, and static analysis)

## 🧪 Testing & Quality Assurance
Safety is our top priority. Our CI/CD pipeline enforces:
*   **Static Analysis:** 100% compliance with mandatory MISRA C rules via `cppcheck`.
*   **Unit Testing:** Fully automated test suites.
*   **Coverage:** 100% Statement and Branch Coverage, with **MC/DC (Modified Condition/Decision Coverage)** applied to safety-critical decision logic (ASIL B adherence).

## 📚 Documentation
All detailed engineering artifacts are documented in our[Project Wiki](https://github.com/seu-usuario/aeb-project/wiki). 

*[System Requirements & Traceability Matrix](https://github.com/seu-usuario/aeb-project/wiki/System-Requirements)
*   [Software Architecture & Stateflow](https://github.com/seu-usuario/aeb-project/wiki/Architecture)
*   [Verification & Validation Plan (Testing Scenarios)](https://github.com/seu-usuario/aeb-project/wiki/Validation-Plan)

## 🚀 How to Build and Run Tests
```bash
# Clone the repository
git clone https://github.com/seu-usuario/aeb-project.git
cd aeb-project

# Compile the tests (Requires GCC)
gcc -O0 --coverage test/test_aeb_logic.c src/aeb_logic.c -o aeb_test

# Run the test suite
./aeb_test

# Generate Coverage Report
gcov src/aeb_logic.c