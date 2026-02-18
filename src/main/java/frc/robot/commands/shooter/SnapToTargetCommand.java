package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;

/** Snap the robot to face the target, adjust hood, and spin up flywheel while button held */
public class SnapToTargetCommand {

    private final Drive drive;
    private final Hood hood;
    private final Shooter shooter;

    public SnapToTargetCommand(Drive drive, Hood hood, Shooter shooter) {
        this.drive = drive;
        this.hood = hood;
        this.shooter = shooter;
        addRequirements(drive, hood, shooter);
    }

    @Override
    public void initialize() {
        ShotCalculator.getInstance().clearCache();
    }

    @Override
    public void execute() {
        // Calculate latest shot parameters
        var shot = ShotCalculator.getInstance().calculateShot();
        if (!shot.isValid()) {
            return; // Skip if target out of range
        }

        // Rotate robot toward target (yaw)
        Rotation2d desiredYaw = shot.robotYaw();
        drive.setTargetYaw(desiredYaw); // drivetrain subsystem should implement PID yaw control

        // Set hood angle
        hood.requestAngle(shot.hoodAngle());

        // Set flywheel speed
        shooter.setFlywheelSpeed(shot.flywheelSpeedRotPerSec());
    }

    @Override
    public boolean isFinished() {
        // Never finishes on its own; ends when button released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally stop rotation and flywheel if you want when button released
        drive.stopYawControl();
        shooter.stopFlywheel();
    }
}
