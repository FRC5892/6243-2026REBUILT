package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter
 *
 * <p>High-level subsystem logic. This class: - Does NOT know about hardware - Does NOT know about
 * tunables - Only talks to ShooterIO
 */
public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;

    // Default behavior:
    //  - Hold idle speed
    //  - Feeder off
    setDefaultCommand(
        Commands.run(
            () -> {
              io.setShooterSpeed(io.getIdleSpeed(), false);
              io.setFiring(false);
            },
            this));
  }

  public void setShooterSpeed(double velocity, boolean firingBoost) {
    io.setShooterSpeed(velocity, firingBoost);
  }

  public void setFiring(boolean firing) {
    io.setFiring(firing);
  }

  public boolean isAtSpeed(double velocity) {
    return io.isAtSpeed(velocity);
  }

  public double getShooterVelocity() {
    return io.getShooterVelocity();
  }

  public double getFeederCurrent() {
    return io.getFeederCurrent();
  }

  @Override
  public void periodic() {
    io.periodic();
  }
}
