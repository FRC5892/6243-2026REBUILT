package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableMeasure;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Flywheel extends SubsystemBase {

  private final LoggedTalonFX topMotor;
  private final LoggedTalonFX bottomMotor;

  private final MotionMagicVelocityTorqueCurrentFOC topMMControl =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicVelocityTorqueCurrentFOC bottomMMControl =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  @Getter @AutoLogOutput private boolean topAtSetpoint = false;
  @Getter @AutoLogOutput private boolean bottomAtSetpoint = false;

  private final LoggedTunableMeasure<MutAngularVelocity> tolerance =
      new LoggedTunableMeasure<>("Flywheel/Tolerance", RPM.mutable(5));

  public Flywheel(LoggedTalonFX topMotor, LoggedTalonFX bottomMotor) {
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    setDefaultCommand(aimCommand());
  }

  public void setSetpoints(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    topMotor.setControl(topMMControl.withVelocity(topVelocity));
    bottomMotor.setControl(bottomMMControl.withVelocity(bottomVelocity));
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
    topMotor.periodic();
    bottomMotor.periodic();

    // Replace atSetpoint with a tolerance check manually
    topAtSetpoint =
        Math.abs(
                topMotor.getVelocity().in(RotationsPerSecond)
                    - topMMControl.getVelocityMeasure().in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    bottomAtSetpoint =
        Math.abs(
                bottomMotor.getVelocity().in(RotationsPerSecond)
                    - bottomMMControl.getVelocityMeasure().in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    ShotCalculator.getInstance().clearCache();
  }
}
