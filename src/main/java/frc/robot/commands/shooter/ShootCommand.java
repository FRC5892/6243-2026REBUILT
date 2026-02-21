package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ShootCommand {

    /** 
     * Creates a functional command to handle shooting. 
     * @param shooter the Shooter subsystem
     * @param indexer the Indexer subsystem
     * @param voltage the fixed voltage to run the indexer at
     * @return a Command ready to be scheduled
     */
    public static Command shoot(Shooter shooter, Indexer indexer, double voltage) {
        return shooter.getFlywheel().run(
            () -> {
                // Calculate shot
                var shot = frc.robot.subsystems.shooter.ShotCalculator.getInstance().calculateShot();

                // Apply flywheel and hood setpoints
                shooter.getFlywheel().setSetpoints(
                    RotationsPerSecond.of(shot.flywheelSpeedRotPerSec()),
                    RotationsPerSecond.of(shot.flywheelSpeedRotPerSec())
);
                shooter.getHood().requestAngle(shot.hoodAngle());

                // Only run indexer if flywheel and hood are ready, and robot is facing correct rotation
                boolean ready = shooter.isReadyToShoot();

                if (ready) {
                    indexer.setVoltage(voltage);
                } else {
                    indexer.setVoltage(0);
                }
            }
        );
    }
}