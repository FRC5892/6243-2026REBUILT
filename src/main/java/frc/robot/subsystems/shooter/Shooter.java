package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hardware-agnostic Shooter subsystem. Talks only to a ShooterIO implementation, never touches
 * motors directly.
 */
public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;

    // Default command: maintain idle speed and stop firing
    setDefaultCommand(
        Commands.run(
            () -> {
              io.setShooterSpeed(io.getIdleSpeed(), false);
              io.setFiring(false);
            },
            this));
  }

  public void setShooterSpeed(double speed, boolean firingBoost) {
    io.setShooterSpeed(speed, firingBoost);
  }

  public void setFiring(boolean firing) {
    io.setFiring(firing);
  }

  public boolean isAtSpeed(double speed) {
    return io.isAtSpeed(speed);
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
