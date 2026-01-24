package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ShooterIOSim implements ShooterIO {

  // Identified flywheel model
  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.identifyVelocitySystem(
              0.02, // kV (volts per rad/s) — tune later
              0.001 // kA (volts per rad/s^2) — tune later
              ),
          DCMotor.getNEO(1),
          1.0 // gearing
          );

  private double appliedFlywheelVoltage = 0.0;
  private double appliedFeederVoltage = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Apply motor voltage
    flywheelSim.setInput(appliedFlywheelVoltage);

    // Advance simulation by 20ms
    flywheelSim.update(0.020);

    // Populate inputs for AdvantageKit
    inputs.flywheelVelocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.feederCurrentAmps = Math.abs(appliedFeederVoltage) * 2.0;

    // Simulate battery sag
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    appliedFlywheelVoltage = volts;
  }

  @Override
  public void setFeederVoltage(double volts) {
    appliedFeederVoltage = volts;
  }
}
