package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** AdvantageKit-compliant wrapper around ShooterIOInputs */
@AutoLog
public class ShooterIOInputsAutoLogged implements LoggableInputs {

  // Logged fields
  public double flywheelVelocityRadPerSec = 0.0;
  public double flywheelCurrentAmps = 0.0;
  public boolean flywheelConnected = true;

  public double feederCurrentAmps = 0.0;
  public boolean feederConnected = true;

  /** Copies values from raw IO object */
  public void setFromRaw(ShooterIO.ShooterIOInputs raw) {
    flywheelVelocityRadPerSec = raw.flywheelVelocityRadPerSec;
    flywheelCurrentAmps = raw.flywheelCurrentAmps;
    flywheelConnected = raw.flywheelConnected;

    feederCurrentAmps = raw.feederCurrentAmps;
    feederConnected = raw.feederConnected;
  }

  /** LoggableInputs interface method */
  @Override
  public void toLog() {
    // AdvantageKit automatically logs public fields; nothing else needed
  }
}
