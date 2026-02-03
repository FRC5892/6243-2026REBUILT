package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class FlywheelIOReal implements FlywheelIO {
  private final SparkMax motor;

  public FlywheelIOReal(int motorId) {
    motor = new SparkMax(motorId, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (motor == null) return;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (motor == null) return;
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
