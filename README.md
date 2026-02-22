# (tbd) (FRC 6243) - 2026 REBUILT

Welcome to the repository for (tbd), FRC Team 6243 Energy NERDs' robot for the 2026 REBUILT season.

## About Team 6243
Team 6243, the Energy NERDs, is an FRC team based out of Energy Institute High School located in Houston, Texas. It is the sister team of FRC 5892, the Energy HEROs.

## About (tbd) (2026)
(tbd) is our robot entry for the 2026 FRC competition, REBUILT. (tbd) features a swerve drivechain (a system allowing wheels to rotate and move in any direction), a dual shooter with a conjoined hood, an expandable hopper design with a roller floor, a slapdown intake, a split indexer, L3? climbing capabilities, and can navigate both over the bump and under the trench.

## Controls

**Driver Controls**

| Function           | Button / Stick           | Description            | LED Indicator Color  |
|--------------------|--------------------------|------------------------|---|
| Drive (field-oriented) | Left Stick (X/Y)         | Robot translation      | N/A |
| Drive (field-oriented) | Right Stick (X/Y)  | Robot rotation         | N/A |
| Slapdown Intake | B (toggle)           | Raising and lowering the slapdown intake | Green when in lowered position|
| Intake in             | Left bumper (hold down) | Intakes balls into the hopper                       | N/A |
| Intake out                   | Right bumper (hold down)                       | Outtakes balls from the hopper                       | N/A |
| Activate auto align    | A (hold down)     | Activates auto align feature whilst held down                       | Red when active |
| Stow hood                          | X (press)                         | Stows hood for going under trench. Not a toggle.   | Blue when hood is stowed |
| Beach Alert                          | Y (hold)                         | Makes the robot's indicator LEDs rainbow fade. Used to indicate to alliance members that we are beached on a fuel and need assistance | Rainbow fade when active                        |

**Codriver Controls**

| Function           | Button / Stick           | Description            | LED Indicator Color  |
|--------------------|--------------------------|------------------------|---|
|  | Left Stick (X/Y)         |       | N/A |
|  | Right Stick (X/Y)  |          | N/A |
| Slapdown Intake |      (toggle)      | Raising and lowering the slapdown intake | Green when in lowered position|
| Intake in             |  (hold down) | Intakes balls into the hopper                       | N/A |
| Intake out                   |  (hold down)                       | Outtakes balls from the hopper                       | N/A |
| Activate auto align    | A (hold down)     | Activates auto align feature whilst held down                       | Red when active |
| Beach Alert                          | Y (hold down)                         | Makes the robot's indicator LEDs rainbow fade. Used to indicate to alliance members that we are beached on a fuel and need assistance | Rainbow fade when active                        |
|              |               |          |           |
|              |               |          |           |
|              |               |          |           |


---

## CAN IDs

| Device              | CAN ID | Notes                                            |
|---------------------|--------|--------------------------------------------------|
|                     |   #    |                                                  |
| Indexer             |   #    |                                                  |
| Feeder and hopper rollers |   #    |                                                  |
| Climb Arm           |   #    |                                                  |
| Intake Rollers      |   #    |                                                  |
| Intake Slapdown     |   #    |                                                  |
| Front Left Drive    |   #    | Swerve Module                                    |
| Front Left Steer    |   #    | Swerve Module                                    |
| Front Right Drive   |   #    | Swerve Module                                    |
| Front Right Steer   |   #    | Swerve Module                                    |
| Back Left Drive     |   #    | Swerve Module                                    |
| Back Left Steer     |   #    | Swerve Module                                    |
| Back Right Drive    |   #    | Swerve Module                                    |
| Back Right Steer    |   #    | Swerve Module                                    |

---

## LED Indicator Color

| Color And Behavior   | RGB Values | Priority    | Notes                                       |
|---------------------|------------|-------------|---------------------------------------------|
| White (flashing)    | `rgb(255, 255, 255)` |      1       | Indicates overheating motor                                          |
| Rainbow (fade) | varius        |     2        | Indicates to allince members that robot is beached                                           |
| Red (solid)                |   `rgb(255, 0, 0)` |    3         | Indicates auto allign feature is active                                           |
| Green (solid)                |   `rgb(0, 255, 0)`        |     4        | Indicates slapdown intake is in the "down" position                                          |
| Blue (solid)                  |   `rgb(0, 0, 255)`        |      5       |   Indicates that hood is in the "stowed" position                                         |
|    |  |     |                                        |



