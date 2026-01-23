package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterIOReal implements ShooterIO {

  private final MotorController flywheelMotor;
  private final MotorController feederMotor;

  public ShooterIOReal(int flywheelPort, int feederPort) {
    flywheelMotor = new Spark(flywheelPort); // WPILib 2026 safe Spark
    feederMotor = new Spark(feederPort);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelVelocityRadPerSec = 0; // Ideally read from encoder if connected
    inputs.flywheelCurrentAmps = 0; // Read from real motor
    inputs.flywheelConnected = flywheelMotor != null;

    inputs.feederCurrentAmps = 0; // Read from real motor
    inputs.feederConnected = feederMotor != null;
  }

  @Override
  public void setFlywheelVelocity(double velocityRadPerSec) {
    // PID control could go here if using SparkMax; for now just voltage placeholder
    if (flywheelMotor != null) {
      flywheelMotor.set(velocityRadPerSec * 0.001); // placeholder feedforward
    }
  }

  @Override
  public void setFeederVoltage(double volts) {
    if (feederMotor != null) {
      feederMotor.set(volts / 12.0);
    }
  }
}
