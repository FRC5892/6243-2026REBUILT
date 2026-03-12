package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.hood.Hood;

public class SnapToTargetCommand extends Command {

  // Keep snap-to-shot yaw tuning aligned with the current drive aiming behavior.
  private static final double YAW_KP = 5.0;
  private static final double YAW_KD = 0.4;

  private final Drive drive;
  private final Hood hood;
  private final LED led;
  private boolean invalidShotLatched = false;

  private final PIDController yawPID = new PIDController(YAW_KP, 0.0, YAW_KD);

  public SnapToTargetCommand(Drive drive, Shooter shooter, LED led) {
    this.drive = drive;
    this.led = led;
    hood = shooter.getHood();

    // Require the actual subsystems
    addRequirements(
        drive, hood // flywheel already has its own default command
        );

    yawPID.enableContinuousInput(-Math.PI, Math.PI);
    yawPID.setTolerance(Math.toRadians(1.0));
  }

  @Override
  public void initialize() {
    ShotCalculator.getInstance().clearCache();
    yawPID.reset();
    invalidShotLatched = false;
  }

  @Override
  public void execute() {
    var shot = ShotCalculator.getInstance().calculateShot();
    if (!shot.isValid()) {
      // Only trigger the warning when the command first notices the invalid shot.
      if (!invalidShotLatched) {
        yawPID.reset();
        led.triggerInvalidShotFlash();
        invalidShotLatched = true;
      }
      // Intentionally do not command drive here; this avoids forcing an abrupt stop.
      return;
    }

    invalidShotLatched = false;

    Rotation2d desiredYaw = shot.robotYaw();
    Rotation2d currentYaw = drive.getRotation();

    double omega = yawPID.calculate(currentYaw.getRadians(), desiredYaw.getRadians());

    // rotate robot only
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

    // While snapping, this command temporarily owns hood aiming too.
    hood.requestAngle(shot.hoodAngle());
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
