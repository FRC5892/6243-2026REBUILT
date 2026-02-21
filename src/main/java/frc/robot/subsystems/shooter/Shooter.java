package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedDIO.SimDIO;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXFlywheelSim;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import lombok.Getter;

public class Shooter {
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;

  public Shooter(CANBus bus) {
    switch (Constants.currentMode) {
      case REAL -> {
        PhoenixTalonFX leftFlywheel =
            new PhoenixTalonFX(
                25, bus, "FlywheelLeft", new PhoenixTalonFollower(26, MotorAlignmentValue.Aligned));

        PhoenixTalonFX rightFlywheel = new PhoenixTalonFX(26, bus, "FlywheelRight");

        flywheel = new Flywheel(rightFlywheel, leftFlywheel);

        hood =
            new Hood(
                new PhoenixTalonFX(27, bus, "Hood"),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
      }

      case SIM -> {
        TalonFXFlywheelSim leftFlywheel =
            new TalonFXFlywheelSim(
                25,
                bus,
                "FlywheelLeft",
                0.0007567661,
                1 / 1.25,
                new PhoenixTalonFollower(26, MotorAlignmentValue.Aligned));

        TalonFXSimpleMotorSim rightFlywheel =
            new TalonFXSimpleMotorSim(26, bus, "FlywheelRight", 0.0007567661, 1 / 1.25);

        flywheel = new Flywheel(rightFlywheel, leftFlywheel);

        hood =
            new Hood(
                new TalonFXSimpleMotorSim(27, bus, "Hood", 0.0017154536, 1.3),
                SimDIO.fromNT("HoodReverse"),
                SimDIO.fromNT("HoodForward"));
      }

      default -> {
        NoOppTalonFX leftFlywheel = new NoOppTalonFX("FlywheelLeft", 1);
        NoOppTalonFX rightFlywheel = new NoOppTalonFX("FlywheelRight", 1);

        flywheel = new Flywheel(rightFlywheel, leftFlywheel);

        hood =
            new Hood(
                new NoOppTalonFX("Hood", 0),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
      }
    }
  }

  /** Main readiness check. */
  public boolean isReadyToShoot() {
    var shot = ShotCalculator.getInstance().calculateShot();

    if (!shot.isValid()) return false;

    return flywheelMatches(shot.flywheelSpeedRotPerSec())
        && hoodMatches(shot.hoodAngle())
        && rotationMatches(shot.robotYaw());
  }

  /** ---------------- MATCH CHECKS ---------------- */
  private boolean flywheelMatches(double target) {
    double actual = flywheel.getVelocity(); // must exist in Flywheel
    return Math.abs(actual - target) < 100;
  }

  private boolean hoodMatches(Rotation2d target) {
    Rotation2d actual = hood.getAngle(); // must exist in Hood
    return Math.abs(actual.minus(target).getDegrees()) < 1.0;
  }

  private boolean rotationMatches(Rotation2d target) {
    Rotation2d current = RobotState.getInstance().getRobotPosition().getRotation();

    return Math.abs(current.minus(target).getDegrees()) < 2.0;
  }
}
