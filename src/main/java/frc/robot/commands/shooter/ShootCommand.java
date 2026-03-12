package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand {

  /**
   * Creates a functional command to handle shooting.
   *
   * @param shooter the Shooter subsystem
   * @param indexer the Indexer subsystem
   * @return a Command ready to be scheduled
   */
  public static Command shoot(Shooter shooter, Indexer indexer) {
    return shooter
        .getFlywheel()
        .run(
            () -> {
              var shot = frc.robot.subsystems.shooter.ShotCalculator.getInstance().calculateShot();

              shooter.getFlywheel().setSetpoints(RPM.of(shot.flywheelSpeedRPM()));

              shooter.getHood().requestAngle(shot.hoodAngle());
            })
        .alongWith(indexer.runWhenShooterReady(shooter));
  }
}
