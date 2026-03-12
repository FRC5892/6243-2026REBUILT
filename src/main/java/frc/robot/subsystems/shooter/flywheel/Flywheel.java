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
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Flywheel extends SubsystemBase {

  // Single leader motor. Followers (if any) should be configured at the LoggedTalon
  // construction level using PhoenixTalonFollower so the CTRE hardware handles
  // aligning follower outputs.
  private final LoggedTalonFX leaderMotor;

  private final MotionMagicVelocityTorqueCurrentFOC mmControl =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  // Store targets explicitly (this is the important part you were missing)
  private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

  @Getter @AutoLogOutput private boolean atSetpoint = false;

  private final LoggedTunableMeasure<MutAngularVelocity> tolerance =
      new LoggedTunableMeasure<>("Flywheel/Tolerance", RPM.mutable(5));

  /**
   * Construct a Flywheel that commands a single leader motor. Any followers should be configured on
   * the LoggedTalon implementation (see {@code PhoenixTalonFollower}).
   */
  public Flywheel(LoggedTalonFX leaderMotor) {
    this.leaderMotor = leaderMotor;
    setDefaultCommand(aimCommand());
  }

  public void setSetpoints(AngularVelocity velocity) {
    targetVelocity = velocity;
    leaderMotor.setControl(mmControl.withVelocity(velocity));
  }

  public Command aimCommand() {
    return run(
        () -> {
          var shot = ShotCalculator.getInstance().calculateShot();

          double speed = shot.flywheelSpeedRPM();

          setSetpoints(RPM.of(speed));
        });
  }

  /** Manual control command for teleop: supplier should provide a target velocity (rot/s). */
  public Command manualShoot(DoubleSupplier targetRotPerSec) {
    return run(() -> setSetpoints(RotationsPerSecond.of(targetRotPerSec.getAsDouble())));
  }

  public boolean isAtTarget() {
    return atSetpoint;
  }

  @Override
  public void periodic() {
    leaderMotor.periodic();

    atSetpoint =
        Math.abs(
                leaderMotor.getVelocity().in(RotationsPerSecond)
                    - targetVelocity.in(RotationsPerSecond))
            < tolerance.get().in(RotationsPerSecond);

    // Clear AFTER everything uses the cached value
    ShotCalculator.getInstance().clearCache();
  }

  /** Returns the current flywheel velocity in units consistent with ShotCalculator. */
  public double getVelocity() {
    return leaderMotor.getVelocity().in(RotationsPerSecond);
  }

  public boolean hasDisconnectedMotor() {
    return leaderMotor.hasDisconnectedMotor();
  }

  public boolean hasOverheatedMotor(double tempThresholdC) {
    return leaderMotor.hasOverheatedMotor(tempThresholdC);
  }
}
