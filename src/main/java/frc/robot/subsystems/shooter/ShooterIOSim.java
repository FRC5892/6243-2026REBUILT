package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * ShooterIOSim
 *
 * <p>Simulation implementation of ShooterIO. NO hardware dependencies.
 *
 * <p>This file is the TEMPLATE for how real IO should behave.
 */
public class ShooterIOSim implements ShooterIO {

  /* =======================
   * Logged Tunable Numbers
   * =======================
   * These replace ALL constants.
   * They are:
   *  - Live adjustable
   *  - Logged by AdvantageKit
   *  - Shared between sim and real
   */

  private final LoggedTunableNumber idleSpeed = new LoggedTunableNumber("Shooter/IdleSpeed", 5.0);

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Shooter/MaxVelocity", 5000.0);

  private final LoggedTunableNumber fireBoost = new LoggedTunableNumber("Shooter/FireBoost", 300.0);

  private final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Shooter/VelocityTolerance", 50.0);

  private final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("Shooter/FeederVoltage", 8.0);

  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Shooter/MaxAcceleration", 300.0);

  /* =======================
   * Simulated State
   * ======================= */

  private double currentVelocity = 0.0;
  private double targetVelocity = 0.0;
  private double feederCurrent = 0.0;

  /* =======================
   * ShooterIO Implementation
   * ======================= */

  @Override
  public void setShooterSpeed(double velocity, boolean firingBoostActive) {
    // Treat very small values as "stop and idle"
    if (Math.abs(velocity) < 1.0) {
      targetVelocity = idleSpeed.get();
      return;
    }

    double boostedVelocity = velocity;
    if (firingBoostActive) {
      boostedVelocity += fireBoost.get();
    }

    // ---- SPEED LIMIT ----
    // This line intentionally caps shooter speed
    targetVelocity = Math.min(boostedVelocity, maxVelocity.get());
    // ---------------------
  }

  @Override
  public void setFiring(boolean firing) {
    // Fake feeder current draw
    feederCurrent = firing ? feederVoltage.get() * 0.02 : 0.0;
  }

  @Override
  public double getShooterVelocity() {
    return currentVelocity;
  }

  @Override
  public double getFeederCurrent() {
    return feederCurrent;
  }

  @Override
  public boolean isAtSpeed(double target) {
    return Math.abs(currentVelocity - target) < velocityTolerance.get();
  }

  @Override
  public double getIdleSpeed() {
    return idleSpeed.get();
  }

  @Override
  public void periodic() {
    // Simple acceleration-limited velocity simulation
    double error = targetVelocity - currentVelocity;
    double accelStep = maxAcceleration.get() * 0.02; // 20ms loop

    if (Math.abs(error) > accelStep) {
      error = Math.signum(error) * accelStep;
    }

    currentVelocity += error;

    // AdvantageKit logging
    Logger.recordOutput("Shooter/Velocity", currentVelocity);
    Logger.recordOutput("Shooter/TargetVelocity", targetVelocity);
    Logger.recordOutput("Shooter/FeederCurrent", feederCurrent);
  }
}
