package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Wrapper around ShooterIOInputs to make it AdvantageKit-loggable */
@AutoLog
public class ShooterIOInputsAutoLogged extends LoggableInputs {

  // The raw inputs object updated by IO layer
  public final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  // Expose fields for logging
  public double flywheelVelocityRadPerSec = 0.0;
  public double flywheelCurrentAmps = 0.0;
  public boolean flywheelConnected = true;

  public double feederCurrentAmps = 0.0;
  public boolean feederConnected = true;

  /** Copies the raw IO values into the loggable fields */
  @Override
  public void toLog() {
    flywheelVelocityRadPerSec = inputs.flywheelVelocityRadPerSec;
    flywheelCurrentAmps = inputs.flywheelCurrentAmps;
    flywheelConnected = inputs.flywheelConnected;

    feederCurrentAmps = inputs.feederCurrentAmps;
    feederConnected = inputs.feederConnected;
  }
}
