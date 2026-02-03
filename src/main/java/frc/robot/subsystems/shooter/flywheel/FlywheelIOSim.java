package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Simulated Flywheel implementation for Robot simulation */
public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim motorSim;

  public FlywheelIOSim() {
    // CAN ID and bus are arbitrary in sim
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.5, 1), DCMotor.getNEO(1), 0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRadsPerSec = motorSim.getAngularVelocity().in(RadiansPerSecond);
    inputs.appliedVoltage = motorSim.getInputVoltage();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    motorSim.update(0.02);
  }
}
