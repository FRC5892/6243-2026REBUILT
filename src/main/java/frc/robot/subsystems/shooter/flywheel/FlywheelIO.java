package frc.robot.subsystems.shooter.flywheel;

public interface FlywheelIO {
  enum Mode {
    COAST,
    DUTY_CYCLE_BANG_BANG,
    TORQUE_CURRENT_BANG_BANG
  }

  class FlywheelIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  class FlywheelIOOutputs {
    public Mode mode = Mode.COAST;
    public double velocityRadsPerSec = 0.0;
  }

  void updateInputs(FlywheelIOInputs inputs);
  void applyOutputs(FlywheelIOOutputs outputs);
}
