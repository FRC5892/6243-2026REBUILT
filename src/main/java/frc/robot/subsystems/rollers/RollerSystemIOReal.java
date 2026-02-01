package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

/** Real hardware implementation of RollerSystemIO using Phoenix‑6 TalonFX */
public class RollerSystemIOReal implements RollerSystemIO {
  private final TalonFX motor;

  public RollerSystemIOReal(int motorId) {
    motor = new TalonFX(motorId); // default CAN bus
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    // Position/velocity come as rotations/rotations per second
    double rotations = motor.getPosition().getValueAsDouble(); 
    double velocityRps = motor.getVelocity().getValueAsDouble();

    inputs.connected = true;
    inputs.positionRads = rotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = velocityRps * 2.0 * Math.PI;

    // Current signals in amps
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();

    // Temperature (Celsius)
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();

    // Motor output percent (duty cycle)
    inputs.appliedVoltage = motor.getDutyCycle().getValueAsDouble();
  }

  @Override
  public void applyOutputs(RollerSystemIOOutputs outputs) {
    // Open‑loop voltage control via DutyCycleOut
    motor.setControl(new DutyCycleOut(outputs.appliedVoltage));

    // Set brake/coast mode
    var motorConfigs = new MotorOutputConfigs()
        .withNeutralMode(
            outputs.brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    motor.getConfigurator().apply(motorConfigs);
  }
}
