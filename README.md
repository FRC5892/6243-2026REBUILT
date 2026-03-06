# KickBack (FRC 6243) - 2026 REBUILT

[![Build](https://github.com/FRC5892/6243-2026REBUILT/actions/workflows/build.yml/badge.svg?branch=main&event=push)](https://github.com/FRC5892/6243-2026REBUILT/actions/workflows/build.yml)
![Last Commit](https://img.shields.io/github/last-commit/FRC5892/6243-2026REBUILT?color=yellow)

Competition robot code for FRC Team 6243 (Energy NERDs) for the 2026 REBUILT season.

## System Overview

This codebase includes:

- Swerve drive with field-relative control.
- Photonvision for pose estimation using apriltags.
- A third camera that switches between AprilTag and object-detection pipelines.
- Snap-to-target alignment for robot yaw and hood angle.
- Slapdown intake, indexer, feeder, shooter, and single-motor climb subsystems.
- LED state arbitration for status, alerts, and operator feedback.
- PathPlanner auto selection and drivetrain characterization/SysId routines.

## Driver Controls (Fixed Mapping)

Controller USB assignments in code:

- Driver controller: `0`
- Codriver controller: `2`

These controller mappings are intentional and fixed for this robot code configuration.

### Driver (Xbox)

| Control | Action |
|---|---|
| Left stick X/Y | Field-relative translation |
| Right stick X | Rotation |
| `A` (hold) | Snap-to-target align (`SnapToTargetCommand`) |
| `X` (press) | Stow hood |
| `B` (toggle) | Toggle intake deploy/retract |
| Left bumper (hold) | Intake in |
| Right bumper (hold) | Intake out |
| `Y` (hold) | Beach alert LED mode |

### Codriver (Xbox)

| Control | Action |
|---|---|
| Left stick Y | Manual hood adjust (default hood command) |
| `A` (hold) | Snap-to-target align |
| Left bumper (toggle) | Toggle intake deploy/retract |
| `X` (hold) | Intake in |
| `B` (hold) | Intake out |
| Right bumper (hold) | Shoot (flywheel + hood + gated index feed) |
| Right trigger (hold) | Indexer unclog (reverse feed) |
| D-pad up (hold) | Climb up |
| D-pad down (hold) | Climb down |
| `Y` (hold) | Beach alert LED mode |

## LED Priority Table

Higher priority states override lower priority states.

| Priority | State | Output |
|---|---|---|
| 1 | Drive motor overheated | Flashing white `rgb(255,255,255)` |
| 2 | Drive motor disconnected | Flashing orange `rgb(255,165,0)` |
| 3 | Beach alert active | Rainbow animation |
| 4 | Auto-align active | Solid red `rgb(255,0,0)` |
| 5 | Intake down | Solid green `rgb(0,255,0)` |
| 6 | Hood stowed | Solid blue `rgb(0,0,255)` |
| 7 | Default | LEDs off |

## Hardware Addressing Reference

`IMPORTANT`: CAN IDs, bus names, camera names, and IO ports below are integration placeholders/reference values. Validate and update all hardware information before running on the robot.
### RIO CAN Bus Devices

| Device | ID | Notes |
|---|---:|---|
| Climb motor | 13 | `RightClimb` |
| Intake roller | 20 | `IntakeRoller` |
| Slapdown motor | 21 | `Slapdown` |
| Flywheel leader | 26 | `FlywheelRight` (left follows in config) |
| Hood motor | 27 | `Hood` |
| Feeder roller | 30 | `Feeder` |
| Indexer right | 31 | `IndexerRight` |
| Indexer left | 32 | `IndexerLeft` (follower of 31) |

### Swerve CAN Bus Devices (`TunerConstants`)

| Device | ID |
|---|---:|
| Pigeon2 | 13 |
| Front Left Drive / Steer / Encoder | 2 / 6 / 41 |
| Front Right Drive / Steer / Encoder | 3 / 7 / 42 |
| Back Left Drive / Steer / Encoder | 4 / 8 / 43 |
| Back Right Drive / Steer / Encoder | 5 / 9 / 44 |

### Other IO

| Device | Port |
|---|---:|
| LED strip PWM | 0 |
| Hood reverse limit DIO | 1 |
| Hood forward limit DIO | 2 |

## Vision Configuration

- AprilTag cameras: `camera_0`, `camera_1`
- Object camera: `camera_object`
- Object camera pipeline indices:
  - `0`: AprilTag
  - `1`: Object detection
- Code behavior: object camera switches to detection in autonomous and back to AprilTag in teleop.

## Bring-Up Checklist

- Set drivetrain CAN bus name in `src/main/java/frc/robot/generated/TunerConstants.java` (replace `"Default Name"`).
- Confirm all CAN IDs and IO ports match final robot wiring.
- Confirm camera device names and active pipelines match coprocessor configuration.
- Verify hood limit switch polarity and hood angle tuning after mechanical changes.
