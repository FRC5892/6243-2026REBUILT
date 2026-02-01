package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();
  private double setpoint = 0.0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setVelocity(double velocityRadsPerSec) {
    setpoint = velocityRadsPerSec;
    FlywheelIO.FlywheelIOOutputs outputs = new FlywheelIO.FlywheelIOOutputs();
    outputs.mode = FlywheelIO.Mode.TORQUE_CURRENT_BANG_BANG;
    outputs.velocityRadsPerSec = setpoint;
    io.applyOutputs(outputs);
  }

  public boolean atGoal() {
    return Math.abs(inputs.velocityRadsPerSec - setpoint) < 0.05;
  }

  public FlywheelIO.FlywheelIOInputs getInputs() {
    return inputs;
  }
}
