package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  /** Container for all shooter sensor inputs (logging + replay). */
  @AutoLog
  class ShooterIOInputs {
    public double flywheelVelocityRadPerSec = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  /** Updates the input structure with the latest sensor values. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets the flywheel motor voltage. */
  default void setFlywheelVoltage(double volts) {}

  /** Sets the feeder motor voltage. */
  default void setFeederVoltage(double volts) {}
}
