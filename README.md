# KickBack (FRC 6243) - 2026 REBUILT

[![Build](https://github.com/FRC5892/6243-2026REBUILT/actions/workflows/build.yml/badge.svg?branch=main&event=push)](https://github.com/FRC5892/6243-2026REBUILT/actions/workflows/build.yml) ![Last Commit](https://img.shields.io/github/last-commit/FRC5892/6243-2026REBUILT?color=yellow)


Robot code for FRC Team 6243 (Energy NERDs) for the 2026 REBUILT season

## What The Robot Can Do

- Drive with field-relative swerve control.
- Run AprilTag-based pose fusion from two vision cameras.
- Switch a third camera between AprilTag and object-detection pipelines.
- Auto-snap robot yaw to shot heading while commanding hood angle.
- Shoot with flywheel + hood setpointing and shooter-readiness-gated indexing.
- Intake and outtake with a slapdown intake mechanism.
- Run indexer + feeder forward for shooting or reverse for unclogging.
- Operate a single-motor climb (up/down hold commands).
- Run LED priority states for fault/status/operator feedback.
- Select PathPlanner autos through dashboard chooser.
- Run drive characterization and SysId routines from the auto chooser.

## Operator Controls

Controller ports in code:
- Driver controller: USB `0`
- Codriver controller: USB `2`

### Driver (Xbox controller)

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

### Codriver (Xbox controller)

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

## LED States (Priority Order)

Higher rows override lower rows.

| Priority | State | Output |
|---|---|---|
| 1 | Drive motor overheated | Flashing white `rgb(255,255,255)` |
| 2 | Drive motor disconnected | Flashing orange `rgb(255,165,0)` |
| 3 | Beach alert active | Rainbow animation |
| 4 | Auto-align active | Solid red `rgb(255,0,0)` |
| 5 | Intake down | Solid green `rgb(0,255,0)` |
| 6 | Hood stowed | Solid blue `rgb(0,0,255)` |
| 7 | Default | LEDs off |

## CAN / IO Map

### RIO CAN bus devices

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

### Swerve CAN bus devices (`TunerConstants`)

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
- Robot code switches object camera to detection in autonomous and back to AprilTag in teleop.

## Important Setup Notes

- Update `src/main/java/frc/robot/generated/TunerConstants.java` CAN bus name from `"Default Name"` to your actual drivetrain CAN bus name before running on hardware.
- Verify USB controller indexing at each event (`0` driver, `2` codriver in current code).
- If hood behavior changes mechanically, re-check hood angle tuning and limit switch orientation.
