package frc.robot.subsystems.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;

public class RollerSystemIOReal implements RollerSystemIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private static final int BRAKE_MODE = 1; // Brake
  private static final int COAST_MODE = 0; // Coast

  public RollerSystemIOReal(int motorId) {
    motor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected = motor.getLastError() == null;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(RollerSystemIOOutputs outputs) {
    motor.setVoltage(outputs.appliedVoltage);

    // Toggle brake/coast dynamically using 2026 API integers
    motor.setIdleMode(outputs.brakeModeEnabled ? BRAKE_MODE : COAST_MODE);
  }
}
