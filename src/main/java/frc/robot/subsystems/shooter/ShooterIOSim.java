package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

  // Simulated motors
  private double shooterVelocity = 0;
  private double targetVelocity = 0;
  private double feederVoltage = 0;
  private double shooterOutput = 0; // simulated applied output

  private final double maxAcceleration = 300; // units/sec^2, tweak to simulate motor
  private final double shooterCurrentFactor = 0.01; // arbitrary scale for current

  @Override
  public void setShooterSpeed(double speed, boolean firingBoost) {
    double ff = firingBoost ? DEFAULT_FIRE_BOOST : 0;

    // Clamp to max/min shooter speeds
    speed = Math.max(MIN_SHOOTER_SPEED, Math.min(MAX_SHOOTER_SPEED, speed));

    // Stop motor if speed is too low (speed-limiting behavior)
    if (Math.abs(speed) < 1) {
      targetVelocity = 0;
      return;
    }

    // Shuffleboard override simulation
    targetVelocity = speed + ff;

    // Store applied output for logging
    shooterOutput = targetVelocity / MAX_SHOOTER_SPEED; // normalized
  }

  @Override
  public boolean isAtSpeed(double speed) {
    // Simulate near-equality check
    return Math.abs(shooterVelocity - speed) < SHOOTER_VELOCITY_RANGE;
  }

  @Override
  public void setFiring(boolean isFiring) {
    feederVoltage = isFiring ? FEEDER_FIRING_VOLTAGE : 0;
  }

  @Override
  public double getShooterVelocity() {
    return shooterVelocity;
  }

  @Override
  public double getFeederCurrent() {
    // Simulate current as proportional to voltage applied
    return feederVoltage * shooterCurrentFactor;
  }

  @Override
  public void periodic() {
    // Simple first-order ramp to simulate motor acceleration
    double delta = targetVelocity - shooterVelocity;
    if (Math.abs(delta) > maxAcceleration / 50.0) { // assuming 50Hz loop
      delta = Math.signum(delta) * maxAcceleration / 50.0;
    }
    shooterVelocity += delta;

    // AdvantageKit logging
    Logger.recordOutput("Shooter/Main Velocity", shooterVelocity);
    Logger.recordOutput("Shooter/Target Velocity", targetVelocity);
    Logger.recordOutput("Shooter/Applied Output", shooterOutput);
    Logger.recordOutput("Shooter/Firing Voltage", feederVoltage);
    Logger.recordOutput("Shooter/Feeder Current", getFeederCurrent());
  }
}
