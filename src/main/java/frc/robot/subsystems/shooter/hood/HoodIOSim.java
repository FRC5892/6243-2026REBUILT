package frc.robot.subsystems.shooter.hood;

public class HoodIOSim implements HoodIO {
  private double simulatedPosition = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRad = simulatedPosition;
    inputs.velocityRadPerSec = 0.0; // for simplicity
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == Mode.POSITION) {
      simulatedPosition = outputs.positionRad;
    }
  }
}
