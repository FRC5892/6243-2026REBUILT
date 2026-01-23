package frc.robot.subsystems.shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private final FlywheelSim flywheelSim;
  private double feederVolts = 0.0;

  public ShooterIOSim() {
    // WPILib-identified flywheel velocity system
    LinearSystem<N1, N1, N1> flywheelPlant =
        LinearSystemId.identifyVelocitySystem(
            0.02,  // kV (V per rad/s) — placeholder
            0.002  // kA (V per rad/s^2) — placeholder
        );

    flywheelSim =
        new FlywheelSim(
            flywheelPlant,
            DCMotor.getNEO(1),
            1.0 // gearing
        );
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    flywheelSim.update(0.02);

    inputs.flywheelVelocityRadPerSec =
        flywheelSim.getAngularVelocityRadPerSec();
    inputs.flywheelCurrentAmps =
        flywheelSim.getCurrentDrawAmps();

    inputs.feederCurrentAmps = Math.abs(feederVolts);

    inputs.flywheelConnected = true;
    inputs.feederConnected = true;
  }

  @Override
  public void setFlywheelVelocity(double velocityRadPerSec) {
    // Simple feedforward approximation for sim
    flywheelSim.setInputVoltage(velocityRadPerSec * 0.01);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederVolts = volts;
  }
}
