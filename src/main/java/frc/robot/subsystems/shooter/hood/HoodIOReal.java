package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StatusSignal;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;

  public HoodIOReal(int motorId) {
    motor = new TalonFX(motorId);

    // Basic configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kF = 0.0;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusSignal<Double> pos = motor.getPosition();
    StatusSignal<Double> vel = motor.getVelocity();

    // Convert from degrees to radians manually
    inputs.positionRad = Math.toRadians(pos.getValue());
    inputs.velocityRadPerSec = Math.toRadians(vel.getValue());
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case POSITION:
        // Phoenix 6 PositionVoltage expects radians converted to motor ticks
        motor.set(new PositionVoltage(radiansToTicks(outputs.positionRad)));
        break;
      case VOLTAGE:
        motor.setVoltage(outputs.percentOutput);
        break;
    }
  }

  private double radiansToTicks(double rad) {
    return rad / (2.0 * Math.PI) * 2048.0; // 2048 ticks per rotation
  }
}
