package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import org.littletonrobotics.junction.Logger;

public class ShootCommand {

  /**
   * Creates a functional command to handle shooting.
   *
   * @param shooter the Shooter subsystem
   * @param indexer the Indexer subsystem
   * @return a Command ready to be scheduled
   */
  public static Command shoot(Shooter shooter, Indexer indexer) {
    // Add a short telemetry hit so we can verify the command was scheduled when the button
    // is pressed. This helps diagnose whether button bindings are reaching the scheduler.
    Command core =
        shooter
            .getFlywheel()
            .run(
                () -> {
                  var shot = ShotCalculator.getInstance().calculateShot();
                  double targetRpm =
                      shot.isValid()
                          ? shot.flywheelSpeedRPM()
                          : ShotCalculator.flywheelIdleRPM.get();

                  // Fall back to the configured idle speed so the flywheel trims from idle instead
                  // of
                  // dropping out completely when the shot is out of range.
                  shooter.getFlywheel().setSetpoints(RPM.of(targetRpm));
                })
            .alongWith(
                shooter
                    .getHood()
                    .run(
                        () -> {
                          var shot = ShotCalculator.getInstance().calculateShot();

                          // Keep the previous hood target when invalid instead of chasing bad data.
                          if (!shot.isValid()) {
                            return;
                          }

                          // Claim the hood here so its manual/default command cannot fight the shot
                          // target.
                          shooter.getHood().requestAngle(shot.hoodAngle());
                        }),
                indexer.runWhenShooterReady(shooter));

    return Commands.sequence(
        Commands.runOnce(() -> Logger.recordOutput("Shooter/ShootCommandScheduled", true)), core);
  }
}
