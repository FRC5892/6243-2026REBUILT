package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;

    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setArmVoltage(double volts) {}

  default void setRollerVoltage(double volts) {}
}
