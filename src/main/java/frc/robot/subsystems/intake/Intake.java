// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/Roller/IntakeVolts", 8.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Intake/Roller/OuttakeVolts", -8.0);

  private final RollerSystem roller;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Intake(RollerSystemIO rollerIO) {
    this.roller = new RollerSystem("Intake roller", "Intake/Roller", rollerIO);
  }

  public void periodic() {

    roller.periodic();

    double rollerVolts = 0.0;
    switch (goal) {
      case INTAKE -> {
        rollerVolts = rollerIntakeVolts.get();
      }

      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
  }

  public enum Goal {
    INTAKE,
    OUTTAKE,
    STOP
  }
}