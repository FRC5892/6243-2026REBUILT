package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

/** Command to move both climb motors down at the same time. */
public class ClimbDownCommand {

  public static Command create(Climb climb) {
    return climb.startEnd(
        () -> {
          climb.climberLeftDown();
          climb.climberRightDown();
        },
        climb::stopAllCommand);
  }
}
