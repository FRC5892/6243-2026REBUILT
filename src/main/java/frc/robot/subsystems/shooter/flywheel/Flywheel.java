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
    rightMotor.setControl(rightMMControl.withVelocity(rightVelocity));
    leftMotor.setControl(leftMMControl.withVelocity(leftVelocity));
  }

  public Command aimCommand() {
    return run(
        () -> {
          double speed = ShotCalculator.getInstance().calculateShot().flywheelSpeedRotPerSec();

          // Both flywheels controlled independently (can be changed later if you want offsets)
          setSetpoints(RotationsPerSecond.of(speed), RotationsPerSecond.of(speed));
        });
  }

  @Override
  public void periodic() {
    rightMotor.periodic();
    leftMotor.periodic();

    // Replace atSetpoint with a tolerance check manually
    rightAtSetpoint =
        Math.abs(
                rightMotor.getVelocity().in(RotationsPerSecond)
                    - rightMMControl.getVelocityMeasure().in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    leftAtSetpoint =
        Math.abs(
                leftMotor.getVelocity().in(RotationsPerSecond)
                    - leftMMControl.getVelocityMeasure().in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    ShotCalculator.getInstance().clearCache();
  }
}
