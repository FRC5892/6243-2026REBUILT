package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;

  public HoodIOReal(int motorId) {
    motor = new TalonFX(motorId);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Phoenix 6 reports rotations and rotations/sec
    double rotations = motor.getPosition().getValueAsDouble();
    double velocityRps = motor.getVelocity().getValueAsDouble();

    inputs.positionRad = rotations * 2.0 * Math.PI;
    inputs.velocityRadPerSec = velocityRps * 2.0 * Math.PI;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case POSITION: {
        // radians → rotations
        double targetRot = outputs.positionRad / (2.0 * Math.PI);

        PositionDutyCycle request =
            new PositionDutyCycle(targetRot)
                .withFeedForward(outputs.dutyCycleFeedforward);

        motor.setControl(request);
        break;
      }

      case VOLTAGE:
        motor.setControl(new DutyCycleOut(outputs.voltagePercent));
        break;

      default:
        motor.setControl(new DutyCycleOut(0.0));
        break;
    }
  }
}
