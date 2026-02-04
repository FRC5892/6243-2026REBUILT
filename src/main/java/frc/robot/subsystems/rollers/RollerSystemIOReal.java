package frc.robot.subsystems.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;

public class RollerSystemIOReal implements RollerSystemIO {

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  public RollerSystemIOReal(int motorId) {
    motor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = motor.getEncoder();
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
  }

  // optional helper method like FlywheelIOReal
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
