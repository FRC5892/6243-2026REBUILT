package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Flywheel extends SubsystemBase {

  private final LoggedTalonFX rightMotor;
  private final LoggedTalonFX leftMotor;

  private final MotionMagicVelocityTorqueCurrentFOC rightMMControl =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicVelocityTorqueCurrentFOC leftMMControl =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Store targets explicitly (this is the important part you were missing)
  private AngularVelocity targetRightVelocity = RotationsPerSecond.of(0);
  private AngularVelocity targetLeftVelocity = RotationsPerSecond.of(0);

  @Getter @AutoLogOutput private boolean rightAtSetpoint = false;
  @Getter @AutoLogOutput private boolean leftAtSetpoint = false;

  private final LoggedTunableMeasure<MutAngularVelocity> tolerance =
      new LoggedTunableMeasure<>("Flywheel/Tolerance", RPM.mutable(5));

  public Flywheel(LoggedTalonFX rightMotor, LoggedTalonFX leftMotor) {
    this.rightMotor = rightMotor;
    this.leftMotor = leftMotor;
    setDefaultCommand(aimCommand());
  }

  public void setSetpoints(AngularVelocity rightVelocity, AngularVelocity leftVelocity) {
    targetRightVelocity = rightVelocity;
    targetLeftVelocity = leftVelocity;

    rightMotor.setControl(rightMMControl.withVelocity(rightVelocity));
    leftMotor.setControl(leftMMControl.withVelocity(leftVelocity));
  }

  public Command aimCommand() {
    return run(
        () -> {
          var shot = ShotCalculator.getInstance().calculateShot();

          double speed = shot.flywheelSpeedRotPerSec();

          setSetpoints(RotationsPerSecond.of(speed), RotationsPerSecond.of(speed));
        });
  }

  public boolean isAtTarget() {
    return rightAtSetpoint && leftAtSetpoint;
  }

  @Override
  public void periodic() {
    rightMotor.periodic();
    leftMotor.periodic();

    rightAtSetpoint =
        Math.abs(
                rightMotor.getVelocity().in(RotationsPerSecond)
                    - targetRightVelocity.in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    leftAtSetpoint =
        Math.abs(
                leftMotor.getVelocity().in(RotationsPerSecond)
                    - targetLeftVelocity.in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    // Clear AFTER everything uses the cached value
    ShotCalculator.getInstance().clearCache();
  }

  /** Returns the current flywheel velocity in units consistent with ShotCalculator. */
  public double getVelocity() {
    return rightMotor.getVelocity().in(RotationsPerSecond);
  }
}
