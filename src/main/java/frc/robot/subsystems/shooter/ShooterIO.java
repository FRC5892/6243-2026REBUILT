package frc.robot.subsystems.shooter;

/**
 * Hardware-agnostic shooter interface. Can be implemented with real motors (REV/CANSparkMax) or
 * simulated for AdvantageKit.
 */
public interface ShooterIO {

  /** Set the main shooter speed. */
  void setShooterSpeed(double speed, boolean firingBoost);

  /** Returns current shooter velocity. */
  double getShooterVelocity();

  /** Returns current drawn by the feeder. */
  double getFeederCurrent();

  /** Controls feeder voltage for firing. */
  void setFiring(boolean isFiring);

  /** Returns true if shooter is at target speed. */
  boolean isAtSpeed(double speed);

  /** Called periodically for logging / simulation updates. */
  void periodic();

  /** Provides the default idle speed for this implementation. */
  default double getIdleSpeed() {
    return 0; // override in real or sim implementations
  }
}
