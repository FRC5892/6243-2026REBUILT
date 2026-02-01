package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getNeo550(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, 0.025, 1), motorModel);

  private PIDController controller = new PIDController(0.001, 0, 0, Constants.loopPeriodSecs);
  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    appliedVolts = motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    double measured = sim.getAngularVelocityRadPerSec();
    double volts = controller.calculate(measured, outputs.velocityRadsPerSec);
    currentOutput = MathUtil.clamp(volts / 12.0, -1.0, 1.0);
  }
}
