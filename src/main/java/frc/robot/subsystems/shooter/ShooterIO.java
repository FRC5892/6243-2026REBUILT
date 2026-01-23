package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    // Flywheel
    public double flywheelVelocityRadPerSec = 0.0;
    public double flywheelCurrentAmps = 0.0;
    public boolean flywheelConnected = true;

    // Feeder
    public double feederCurrentAmps = 0.0;
    public boolean feederConnected = true;
  }

  /** Update all sensor inputs */
  void updateInputs(ShooterIOInputs inputs);

  /** Flywheel closed-loop velocity control (hardware-specific) */
  void setFlywheelVelocity(double velocityRadPerSec);

  /** Feeder open-loop voltage control */
  void setFeederVoltage(double volts);
}
