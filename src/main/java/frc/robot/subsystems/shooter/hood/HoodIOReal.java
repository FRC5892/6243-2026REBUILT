package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;

  public HoodIOReal(int motorId) {
    motor = new TalonFX(motorId);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Phoenix 6 TalonFX: getPosition() and getVelocity() already return doubles in native units
    // Convert to radians if needed (assuming 1 rotation = 2π radians)
    inputs.positionRad = motor.getPosition().getValue() * (2 * Math.PI);
    inputs.velocityRadPerSec = motor.getVelocity().getValue() * (2 * Math.PI);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case POSITION:
        // Use PositionVoltage to set position in radians
        motor.set(new PositionVoltage(outputs.positionRad));
        break;
      case VOLTAGE:
        // For open-loop voltage control, just set percent output
        motor.setVoltage(outputs.voltage);
        break;
    }
  }
}
