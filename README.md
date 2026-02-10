# N-EBCM

A miniature Anti-lock Braking System (ABS) demonstration project designed to experiment with safety-critical software/firmware. Runs on an Infineon AURIX TC375 Lite Kit and controls a single brake rotor test rig.

## Overview

This project explores basic principles of safety-critical embedded systems by implementing a functional ABS on a small-scale brake setup. The AURIX TC375's multi-core architecture provides seemingly countless facilities for safe hardware operation and software execution. I'm thinking I'll work on a CAN or SPI bootloader at some point in the future so that the application can be flashed via another MCU (I have an S32K I enjoy using).

## Core Assignment

| Core | Function | Criticality |
|------|----------|-------------|
| Core 0 | ABS algorithm and brake actuation | Safety-critical |
| Core 1 | Motor control | Safety-critical |
| Core 2 | Ultrasonic sensor DSP (Crude ADAS), diagnostics, debug output | Non-safety |

## Hardware

- **MCU**: Infineon AURIX TC375 Lite Kit
- **Brake**: Single brake rotor test rig
- **Sensors**: Ultrasonic sensor (DSP on Core 2)
- Additional hardware details TBD

## Architecture

The system leverages the TC375's lockstep cores and memory protection features to maintain isolation between safety-critical ABS/motor control operations and non-critical diagnostic functions.

## Safety Framework

### Safe Software Startup (SSW)

Before the application runs, the system executes a comprehensive suite of hardware self-tests to verify the MCU is functioning correctly:

| Test | Description |
|------|-------------|
| **LBIST** | Logic Built-In Self-Test - verifies CPU core logic integrity via hardware scan chains |
| **MONBIST** | Monitor BIST - tests power supply monitoring circuitry |
| **MCU FW Check** | Verifies boot ROM executed correctly by checking safety-critical register values |
| **MCU Startup** | Validates flash protection and security configuration via CRC |
| **SMU Alive Test** | Verifies Safety Management Unit is functional |
| **Register Monitor Test** | Tests safety flip-flops across hardware modules (MTU, DMA, PLL, CCU, etc.) |
| **MBIST** | Memory Built-In Self-Test - tests all on-chip SRAM integrity |
| **Lockstep Injection** | Injects a fault to verify the lockstep comparator detects mismatches |

### Vital Framework (VFW)

The Vital Framework provides runtime execution path integrity monitoring:

- **Checkpoint System**: Critical functions register checkpoints. These checkpoints are used to verify the integrity of the execution path.
- **MPU Protection**: VFW data is protected via hardware Memory Protection Unit. The framework uses its own protection set with exclusive R/W access to the `vfw_safe0` memory region, while application code has read-only access.
- **Stack Monitoring**: Stack painting enables high-watermark detection for stack overflow analysis.
- **Safety Response**: On integrity failure, interrupts are disabled and the system enters a safe state.

### Startup Flow

```
core0_main()
 ├── EbcmSsw_runAppSwStartup()
 │    ├── LBIST (may trigger warm reset)
 │    ├── MONBIST
 │    ├── MCU FW Check
 │    ├── MCU Startup Check
 │    ├── SMU Alive Test
 │    ├── Register Monitor Test
 │    └── MBIST
 ├── Lockstep Injection Test
 ├── VFW_Init() (MPU + checkpoint registry)
 ├── Hardware Init (LEDs, sensors, watchdog)
 └── Main Loop
```
