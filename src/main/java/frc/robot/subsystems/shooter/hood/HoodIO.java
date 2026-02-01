package frc.robot.subsystems.shooter.hood;

public interface HoodIO {

  class HoodIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
  }

  class HoodIOOutputs {
    public double positionRad = 0.0;          // radians for POSITION mode
    public double voltagePercent = 0.0;       // [-1..1] for VOLTAGE mode
    public double dutyCycleFeedforward = 0.0; // optional feedforward for POSITION mode
    public Mode mode = Mode.POSITION;         // default mode
  }

  enum Mode {
    POSITION, // closed-loop position control
    VOLTAGE   // simple open-loop voltage
  }

  void updateInputs(HoodIOInputs inputs);

  void applyOutputs(HoodIOOutputs outputs);
}
