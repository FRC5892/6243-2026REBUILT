package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
              // Calculate shot
              var shot = frc.robot.subsystems.shooter.ShotCalculator.getInstance().calculateShot();

              // Apply flywheel and hood setpoints
              shooter
                  .getFlywheel()
                  .setSetpoints(
                      RotationsPerSecond.of(shot.flywheelSpeedRotPerSec()),
                      RotationsPerSecond.of(shot.flywheelSpeedRotPerSec()));
              shooter.getHood().requestAngle(shot.hoodAngle());

              // Only run indexer if flywheel and hood are ready, and robot is facing correct
              // rotation
              if (shooter.isReadyToShoot()) {
                // Run feeder + indexer rollers forward
                indexer.runForShooting().schedule();
              } else {
                // Stop all indexer rollers if not ready
                indexer.stopAll().schedule();
              }
            });
  }
}
