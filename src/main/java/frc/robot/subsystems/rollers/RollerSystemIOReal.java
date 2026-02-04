package frc.robot.subsystems.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.util.Units;

public class RollerSystemIOReal implements RollerSystemIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public RollerSystemIOReal(int motorId) {
    motor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = motor.getEncoder();

    // Initial motor configuration
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);

    // Apply config (2026 API)
    motor.configure(config);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected = motor.getLastError() == null;
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(RollerSystemIOOutputs outputs) {
    motor.setVoltage(outputs.appliedVoltage);

    // Toggle brake/coast at runtime
    SparkMaxConfig runtimeConfig = new SparkMaxConfig();
    runtimeConfig.idleMode(
        outputs.brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake
                                 : SparkBaseConfig.IdleMode.kCoast
    );

    // 2026 API: single-argument configure
    motor.configure(runtimeConfig);
  }
}
