package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;

  public HoodIOReal(int motorId) {
    motor = new TalonFX(motorId);
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // getPosition() returns StatusSignal<Angle>, so convert to radians
    inputs.positionRad = motor.getPosition().getValue().toRadians();
    inputs.velocityRadPerSec = motor.getVelocity().getValue().toRadians();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case POSITION:
        motor.set(new PositionVoltage(outputs.positionRad)); // radians to internal units
        break;
      case VOLTAGE:
        motor.setVoltage(outputs.positionRad); // if using voltage %
        break;
    }
  }
}
