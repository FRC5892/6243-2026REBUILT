package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTalon.FlywheelSim;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

/** Simulated Flywheel implementation for Robot simulation */
public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim motorSim;

  public FlywheelIOSim() {
    // CAN ID and bus are arbitrary in sim
    motorSim = new FlywheelSim(1, CANBus.CAN, "FlywheelSim");
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRadsPerSec = motorSim.getVelocity().get(RadiansPerSecond);
    inputs.appliedVolts = motorSim.getPrimaryAppliedVoltage().get(Volts);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    motorSim.setControl(motorSim.getPrimaryVelocityControlRequest(outputs.velocityRadsPerSec));
  }

  public LoggedTalonFX getMotor() {
    return motorSim;
  }
}
