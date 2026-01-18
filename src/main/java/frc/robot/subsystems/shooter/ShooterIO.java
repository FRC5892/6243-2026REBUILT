package frc.robot.subsystems.shooter;

/**
 * ShooterIO
 *
 * <p>Hardware abstraction layer for the shooter. This interface is implemented by: - ShooterIOSim
 * (simulation) - ShooterIOReal (real robot, NOT included yet)
 *
 * <p>Shooter.java depends ONLY on this interface.
 */
public interface ShooterIO {

  /**
   * Sets the shooter target speed.
   *
   * @param velocity Target velocity (units defined by implementation)
   * @param firingBoost Whether to apply extra feedforward / voltage
   */
  void setShooterSpeed(double velocity, boolean firingBoost);

  /** Enables or disables the feeder */
  void setFiring(boolean firing);

  /**
   * @return current shooter velocity
   */
  double getShooterVelocity();

  /**
   * @return current drawn by feeder motor (or simulated equivalent)
   */
  double getFeederCurrent();

  /** Checks if shooter is within tolerance of a target speed. Implementation defines tolerance. */
  boolean isAtSpeed(double targetVelocity);

  /** Called once per robot loop */
  void periodic();

  /**
   * Exposes idle speed to Shooter.java without Shooter knowing tunables exist. This keeps Shooter
   * logic clean and hardware-agnostic.
   */
  double getIdleSpeed();
}
