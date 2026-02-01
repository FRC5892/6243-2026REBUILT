package frc.robot.subsystems.shooter.hood;

public interface HoodIO {
  class HoodIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
  }

  class HoodIOOutputs {
    public double positionRad = 0.0;
    public Mode mode = Mode.POSITION;
  }

  enum Mode {
    POSITION, // closed-loop position control
    VOLTAGE   // simple open-loop voltage
  }

  void updateInputs(HoodIOInputs inputs);

  void applyOutputs(HoodIOOutputs outputs);
}
