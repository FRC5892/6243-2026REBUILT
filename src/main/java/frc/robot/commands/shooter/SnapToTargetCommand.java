package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;

public class SnapToTargetCommand extends Command {

    private final Drive drive;
    private final Shooter shooter;

    private final PIDController yawPID = new PIDController(6.0, 0.0, 0.2);

    public SnapToTargetCommand(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;

        // Require the actual subsystems
        addRequirements(
            drive,
            shooter.getHood()   // flywheel already has its own default command
        );

        yawPID.enableContinuousInput(-Math.PI, Math.PI);
        yawPID.setTolerance(Math.toRadians(1.0));
    }

    @Override
    public void initialize() {
        ShotCalculator.getInstance().clearCache();
    }

    @Override
    public void execute() {
        var shot = ShotCalculator.getInstance().calculateShot();
        if (!shot.isValid()) return;

        Rotation2d desiredYaw = shot.robotYaw();
        Rotation2d currentYaw = drive.getPose().getRotation();

        double omega = yawPID.calculate(
                currentYaw.getRadians(),
                desiredYaw.getRadians());

        // rotate robot only
        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

        // hood is manual
        shooter.getHood().requestAngle(shot.hoodAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }
}
