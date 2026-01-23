package frc.robot.subsystems.shooter;

import com.revrobotics.SparkMax;
import com.revrobotics.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOReal implements ShooterIO {

  private final SparkMax flywheel;
  private final SparkMax feeder;
  private final SparkClosedLoopController flywheelPID;

  public ShooterIOReal(int flywheelCanId, int feederCanId) {
    flywheel = new SparkMax(flywheelCanId, MotorType.kBrushless);
    feeder = new SparkMax(feederCanId, MotorType.kBrushless);

    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig.closedLoop.pid(0.0003, 0.0, 0.0);
    flywheel.configure(
        flywheelConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
    );

    flywheelPID = flywheel.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean flywheelOk = flywheel.getBusVoltage() > 1.0;
    boolean feederOk = feeder.getBusVoltage() > 1.0;

    inputs.flywheelVelocityRadPerSec =
        flywheel.getEncoder().getVelocity();
    inputs.flywheelCurrentAmps =
        flywheel.getOutputCurrent();
    inputs.feederCurrentAmps =
        feeder.getOutputCurrent();

    inputs.flywheelConnected = flywheelOk;
    inputs.feederConnected = feederOk;
  }

  @Override
  public void setFlywheelVelocity(double velocityRadPerSec) {
    flywheelPID.setReference(
        velocityRadPerSec,
        SparkBase.ControlType.kVelocity
    );
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setVoltage(volts);
  }
}
