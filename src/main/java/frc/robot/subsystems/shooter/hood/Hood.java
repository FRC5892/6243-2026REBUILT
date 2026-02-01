package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
  private double setpoint = 0.0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setPosition(double positionRad) {
    setpoint = positionRad;
    HoodIO.HoodIOOutputs outputs = new HoodIO.HoodIOOutputs();
    outputs.mode = HoodIO.Mode.POSITION;
    outputs.positionRad = setpoint;
    io.applyOutputs(outputs);
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionRad - setpoint) < 0.01; // ~0.5 deg tolerance
  }

  public HoodIO.HoodIOInputs getInputs() {
    return inputs;
  }
}
