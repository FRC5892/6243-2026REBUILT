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
  private static final double MOTOR_OVERHEAT_TEMP_C = 80.0;

  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;

  public Shooter(CANBus bus) {
    switch (Constants.currentMode) {
      case REAL -> {
        // Configure right Talon as leader and add the left Talon as a CTRE follower.
        PhoenixTalonFX rightFlywheel =
            new PhoenixTalonFX(
                26,
                bus,
                "FlywheelRight",
                new PhoenixTalonFollower(25, MotorAlignmentValue.Opposed));

        // Flywheel commands only the leader; followers inherit the output via CTRE.
        flywheel = new Flywheel(rightFlywheel);

        hood =
            new Hood(
                new PhoenixTalonFX(27, bus, "Hood"),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
      }

      case SIM -> {
        // Configure the simulated leader (26) and add the simulated follower (25).
        TalonFXFlywheelSim rightFlywheel =
            new TalonFXFlywheelSim(
                26,
                bus,
                "FlywheelRight",
                0.0007567661,
                1 / 1.25,
                new PhoenixTalonFollower(25, MotorAlignmentValue.Opposed));

        flywheel = new Flywheel(rightFlywheel);

        hood =
            new Hood(
                new TalonFXSimpleMotorSim(27, bus, "Hood", 0.0017154536, 1.3),
                SimDIO.fromNT("HoodReverse"),
                SimDIO.fromNT("HoodForward"));
      }

      default -> {
        // Replay/default: create a leader with one follower (replay doesn't control physical IO)
        NoOppTalonFX rightFlywheel = new NoOppTalonFX("FlywheelRight", 1);
        flywheel = new Flywheel(rightFlywheel);

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

    return flywheelMatches(shot.flywheelSpeedRPM())
        && hoodMatches(shot.hoodAngle())
        && rotationMatches(shot.robotYaw());
  }

  /** ---------------- MATCH CHECKS ---------------- */
  private boolean flywheelMatches(double targetRPM) {
    double actualRPM = flywheel.getVelocity() * 60.0; // getVelocity() is rot/s
    return Math.abs(actualRPM - targetRPM) < 100;
  }

  private boolean hoodMatches(Rotation2d target) {
    Rotation2d actual = hood.getAngle(); // must exist in Hood
    return Math.abs(actual.minus(target).getDegrees()) < 1.0;
  }

  private boolean rotationMatches(Rotation2d target) {
    Rotation2d current = RobotState.getInstance().getRobotPosition().getRotation();

    return Math.abs(current.minus(target).getDegrees()) < 2.0;
  }

  /** Returns true if any shooter motor is disconnected. */
  public boolean hasDisconnectedMotor() {
    return flywheel.hasDisconnectedMotor() || hood.hasDisconnectedMotor();
  }

  /** Returns true if any shooter motor is above the overheat threshold. */
  public boolean hasOverheatedMotor() {
    return flywheel.hasOverheatedMotor(MOTOR_OVERHEAT_TEMP_C)
        || hood.hasOverheatedMotor(MOTOR_OVERHEAT_TEMP_C);
  }
}
