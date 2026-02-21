package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

/** Command to move both climb motors up at the same time. */
public class ClimbUpCommand {

  public static Command create(Climb climb) {
    return climb.startEnd(
        () -> {
          climb.climberLeftUp();
          climb.climberRightUp();
        },
        climb::stopAllCommand);
  }
}
