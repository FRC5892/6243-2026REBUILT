package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class FlywheelIOReal implements FlywheelIO {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController controller;

  private double lastVoltage = 0.0;

  private static final double kP = 0.001;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  public FlywheelIOReal(int deviceId) {
    motor = new CANSparkMax(deviceId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.enableVoltageCompensation(12.0);
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    encoder = motor.getEncoder();
    controller = new PIDController(kP, kI, kD);
    controller.setTolerance(0.05);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected = motor.getFirmwareVersion() > 0;
    inputs.positionRads = Math.toRadians(encoder.getPosition() * 360.0);
    inputs.velocityRadsPerSec = Math.toRadians(encoder.getVelocity() * 2 * Math.PI / 60.0);
    inputs.appliedVoltage = lastVoltage;
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    double measured = Math.toRadians(encoder.getVelocity() * 2 * Math.PI / 60.0);
    double volts = controller.calculate(measured, outputs.velocityRadsPerSec);
    volts = Math.max(-12.0, Math.min(12.0, volts));
    motor.setVoltage(volts);
    lastVoltage = volts;
  }
}
