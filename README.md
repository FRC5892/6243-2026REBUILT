
# FRC 6243 - 2026 REBUILT

<p align="center">
  <b>KickBack - FRC Team 6243</b><br>
  <i>2026 Competition Robot Code</i>
</p>

<p align="center">
  <a href="https://github.com/FRC5892/6243-2026REBUILT/actions/workflows/build.yml"><img src="https://github.com/FRC5892/6243-2026REBUILT/actions/workflows/build.yml/badge.svg?branch=main&event=push" alt="Build Status"></a>
  <a href="https://img.shields.io/github/last-commit/FRC5892/6243-2026REBUILT?color=yellow"><img src="https://img.shields.io/github/last-commit/FRC5892/6243-2026REBUILT?color=yellow" alt="Last Commit"></a>
  <a href="https://github.com/FRC5892/6243-2026REBUILT/blob/main/WPILib-License.md"><img src="https://img.shields.io/badge/license-WPILib%20%2B%20AdvantageKit-blue" alt="License"></a>
  <a href="https://adoptium.net/"><img src="https://img.shields.io/badge/java-17-informational" alt="Java 17"></a>
  <a href="https://github.com/FRC5892/6243-2026REBUILT/blob/main/build.gradle"><img src="https://img.shields.io/badge/build-Gradle-02303A" alt="Gradle"></a>
</p>

---

## Project Overview

This repository contains the official codebase for FRC Team 6243 (Energy NERDs) and the 2026 robot, KickBack. The project leverages modern FRC technologies and best practices for reliability, performance, and maintainability.


**Key Features:**

- WPILib command-based structure
- AdvantageKit logging and dashboard integration
- CTRE Phoenix 6 motor control
- PathPlanner auto routines
- Swerve drive with field-relative control
- AprilTag pose fusion (PhotonVision)
- Dual shooter with dynamic hood/flywheel targets
- Tunable backrollers for shot control
- Robust subsystem separation: drive, intake, indexer, shooter, LEDs, vision
- Comprehensive auto chooser and drive characterization

---

## Status & Activity

[![Open Issues](https://img.shields.io/github/issues/FRC5892/6243-2026REBUILT)](https://github.com/FRC5892/6243-2026REBUILT/issues)
[![Open PRs](https://img.shields.io/github/issues-pr/FRC5892/6243-2026REBUILT)](https://github.com/FRC5892/6243-2026REBUILT/pulls)

---

## Controls & Operator Layout

**Controller USB Ports:**

- Driver controller: `0`
- Codriver controller: `2`

Mappings are defined in [`src/main/java/frc/robot/RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java).

### Driver

| Control | Action |
|---|---|
| Left stick X/Y | Field-relative translation |
| Right stick X | Rotation |
| `A` (hold) | Snap-to-target (yaw + hood alignment) |
| `X` (press) | Stow hood |
| `B` (toggle) | Toggle intake deploy/retract |
| Left bumper (hold) | Intake in |
| Right bumper (hold) | Intake out |
| `Y` (hold) | Beach alert LED mode |

### Codriver

| Control | Action |
|---|---|
| Left stick Y | Manual hood adjust (default hood command) |
| `A` (hold) | Snap-to-target (yaw + hood alignment) |
| Left bumper (toggle) | Toggle intake deploy/retract |
| `X` (hold) | Intake in |
| `B` (hold) | Intake out |
| Right bumper (hold) | Shoot (flywheel + hood request + gated feed) |
| Right trigger (hold) | Indexer unclog (reverse feed) |
| `Y` (hold) | Beach alert LED mode |

---

## LED Indicator Priority

Higher rows override lower rows.

| Priority | State | Output |
|---|---|---|
| 1 | Drive motor overheated | Flashing white `rgb(255,255,255)` |
| 2 | Drive motor disconnected | Flashing orange `rgb(255,165,0)` |
| 3 | Beach alert active | Rainbow fade animation |
| 4 | Invalid shot | Flashing red `rgb(255,0,0)` |
| 5 | Auto-align active | Solid red `rgb(255,0,0)` |
| 6 | Intake down | Solid green `rgb(0,255,0)` |
| 7 | Hood stowed | Solid blue `rgb(0,0,255)` |
| 8 | Default | LEDs TBD |

---

## Hardware Map (Reference)

> **Note:** CAN IDs, bus names, camera names, and IO ports below are reference placeholders. Verify and update to match final robot wiring before running on hardware.

| Device | Bus / IO Type | ID / Port | Function / Notes |
|---|---|---:|---|
| Intake roller | RIO CAN | 20 | `IntakeRoller` |
| Slapdown motor | RIO CAN | 21 | `Slapdown` |
| Flywheel leader | RIO CAN | 26 | `FlywheelRight` (left follows in config) |
| Hood motor | RIO CAN | 27 | `Hood` |
| Feeder roller | RIO CAN | 30 | `Feeder` |
| Indexer right | RIO CAN | 31 | `IndexerRight` |
| Indexer left | RIO CAN | 32 | `IndexerLeft` (follower of 31) |
| Pigeon2 | Swerve CAN (`TunerConstants`) | 13 | Drivetrain gyro |
| Front Left Drive | Swerve CAN (`TunerConstants`) | 2 | Swerve module drive motor |
| Front Left Steer | Swerve CAN (`TunerConstants`) | 6 | Swerve module steer motor |
| Front Left Encoder | Swerve CAN (`TunerConstants`) | 41 | Swerve module azimuth encoder |
| Front Right Drive | Swerve CAN (`TunerConstants`) | 3 | Swerve module drive motor |
| Front Right Steer | Swerve CAN (`TunerConstants`) | 7 | Swerve module steer motor |
| Front Right Encoder | Swerve CAN (`TunerConstants`) | 42 | Swerve module azimuth encoder |
| Back Left Drive | Swerve CAN (`TunerConstants`) | 4 | Swerve module drive motor |
| Back Left Steer | Swerve CAN (`TunerConstants`) | 8 | Swerve module steer motor |
| Back Left Encoder | Swerve CAN (`TunerConstants`) | 43 | Swerve module azimuth encoder |
| Back Right Drive | Swerve CAN (`TunerConstants`) | 5 | Swerve module drive motor |
| Back Right Steer | Swerve CAN (`TunerConstants`) | 9 | Swerve module steer motor |
| Back Right Encoder | Swerve CAN (`TunerConstants`) | 44 | Swerve module azimuth encoder |
| LED strip | PWM | 0 | Addressable LED strip output |
| Hood reverse limit | DIO | 1 | Hood reverse travel limit switch |
| Hood forward limit | DIO | 2 | Hood forward travel limit switch |

---

## Vision System

- AprilTag camera names: `camera_0`, `camera_1`
- Object camera name: `camera_object`
- Real robot stack: two cameras for AprilTag detection plus one for gamepiece detection (switches to AprilTag during teleop)
- Object camera pipeline indices: `0` = AprilTag, `1` = Object detection
- Runtime behavior: `autonomousInit()` sets object camera to detection pipeline, `teleopInit()` sets back to AprilTag pipeline

---

## Simulation & Tuning

This project includes a simulation harness for shot calculation and shooter tuning:

- **Script:** `runShotCalc.sh` (Linux/macOS)
- **Usage:** `./runShotCalc.sh [distance1 distance2 ...]`
  - If no arguments are given, the script sweeps a range of distances.
  - Outputs calculated hood angle and flywheel RPM for each distance.
- **Tuning:** All shooter parameters are exposed as `LoggedTunableNumber` entries under the `ShotTuning/` prefix and can be adjusted live via dashboard tools (AdvantageScope, Shuffleboard, etc.).
- **Live Outputs:** Real-time values (e.g., calculated angles, RPMs, shot validity) are logged under the `ShotCalculator/` and `Hood/` prefixes for easy monitoring and analysis.

---

## Setup Checklist

1. Set drivetrain CAN bus name in [`src/main/java/frc/robot/generated/TunerConstants.java`](src/main/java/frc/robot/generated/TunerConstants.java).
2. Validate all CAN IDs, DIO, and PWM assignments.
3. Confirm vision device names and object-camera pipeline indices.
4. Verify hood limit switch polarity and hood calibration.

### Driver

| Control | Action |
|---|---|
| Left stick X/Y | Field-relative translation |
| Right stick X | Rotation |
| `A` (hold) | Snap-to-target (yaw + hood alignment) |
| `X` (press) | Stow hood |
| `B` (toggle) | Toggle intake deploy/retract |
| Left bumper (hold) | Intake in |
| Right bumper (hold) | Intake out |
| `Y` (hold) | Beach alert LED mode |

### Codriver

| Control | Action |
|---|---|
| Left stick Y | Manual hood adjust (default hood command) |
| `A` (hold) | Snap-to-target (yaw + hood alignment) |
| Left bumper (toggle) | Toggle intake deploy/retract |
| `X` (hold) | Intake in |
| `B` (hold) | Intake out |
| Right bumper (hold) | Shoot (flywheel + hood request + gated feed) |
| Right trigger (hold) | Indexer unclog (reverse feed) |
| `Y` (hold) | Beach alert LED mode |

## LED Indicator Priority

Higher rows override lower rows.

| Priority | State | Output |
|---|---|---|
| 1 | Drive motor overheated | Flashing white `rgb(255,255,255)` |
| 2 | Drive motor disconnected | Flashing orange `rgb(255,165,0)` |
| 3 | Beach alert active | Rainbow fade animation |
| 4 | Invalid shot | Flashing red `rgb(255,0,0)` |
| 5 | Auto-align active | Solid red `rgb(255,0,0)` |
| 6 | Intake down | Solid green `rgb(0,255,0)` |
| 7 | Hood stowed | Solid blue `rgb(0,0,255)` |
| 8 | Default | LEDs TBD |

## Hardware Map (Reference)

`IMPORTANT`: CAN IDs, bus names, camera names, and IO ports below are reference placeholders. Verify and update to match final robot wiring before running on hardware.

| Device | Bus / IO Type | ID / Port | Function / Notes |
|---|---|---:|---|
| Intake roller | RIO CAN | 20 | `IntakeRoller` |
| Slapdown motor | RIO CAN | 21 | `Slapdown` |
| Flywheel leader | RIO CAN | 26 | `FlywheelRight` (left follows in config) |
| Hood motor | RIO CAN | 27 | `Hood` |
| Feeder roller | RIO CAN | 30 | `Feeder` |
| Indexer right | RIO CAN | 31 | `IndexerRight` |
| Indexer left | RIO CAN | 32 | `IndexerLeft` (follower of 31) |
| Pigeon2 | Swerve CAN (`TunerConstants`) | 13 | Drivetrain gyro |
| Front Left Drive | Swerve CAN (`TunerConstants`) | 2 | Swerve module drive motor |
| Front Left Steer | Swerve CAN (`TunerConstants`) | 6 | Swerve module steer motor |
| Front Left Encoder | Swerve CAN (`TunerConstants`) | 41 | Swerve module azimuth encoder |
| Front Right Drive | Swerve CAN (`TunerConstants`) | 3 | Swerve module drive motor |
| Front Right Steer | Swerve CAN (`TunerConstants`) | 7 | Swerve module steer motor |
| Front Right Encoder | Swerve CAN (`TunerConstants`) | 42 | Swerve module azimuth encoder |
| Back Left Drive | Swerve CAN (`TunerConstants`) | 4 | Swerve module drive motor |
| Back Left Steer | Swerve CAN (`TunerConstants`) | 8 | Swerve module steer motor |
| Back Left Encoder | Swerve CAN (`TunerConstants`) | 43 | Swerve module azimuth encoder |
| Back Right Drive | Swerve CAN (`TunerConstants`) | 5 | Swerve module drive motor |
| Back Right Steer | Swerve CAN (`TunerConstants`) | 9 | Swerve module steer motor |
| Back Right Encoder | Swerve CAN (`TunerConstants`) | 44 | Swerve module azimuth encoder |
| LED strip | PWM | 0 | Addressable LED strip output |
| Hood reverse limit | DIO | 1 | Hood reverse travel limit switch |
| Hood forward limit | DIO | 2 | Hood forward travel limit switch |

## Vision

- AprilTag camera names: `camera_0`, `camera_1`
- Object camera name: `camera_object`
- Real robot stack: two cameras for AprilTag detection plus one for gamepiece detection (switches to Apriltag during teleop).
- Object camera pipeline indices:
  - `0`: AprilTag
  - `1`: Object detection
- Runtime behavior:
  - `autonomousInit()`: object camera set to detection pipeline.
  - `teleopInit()`: object camera set back to AprilTag pipeline.

## Setup Checklist

1. Set drivetrain CAN bus name in `src/main/java/frc/robot/generated/TunerConstants.java` (replace `"Default Name"`).
2. Validate all CAN IDs, DIO, and PWM assignments.
3. Confirm vision device names and object-camera pipeline indices.
4. Verify hood limit switch polarity and hood calibration.
