package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  // Tunables live here so they work in replay
  private final LoggedTunableNumber targetVelocityRadPerSec =
      new LoggedTunableNumber("Shooter/TargetVelocityRadPerSec", 4000.0);

  private final LoggedTunableNumber velocityToleranceRadPerSec =
      new LoggedTunableNumber("Shooter/VelocityToleranceRadPerSec", 20.0);

  private final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("Shooter/FeederVoltage", 6.0);

  // Alerts for motor disconnects
  private final Alert flywheelDisconnected =
      new Alert("Shooter flywheel motor disconnected", AlertType.kError);

  private final Alert feederDisconnected =
      new Alert("Shooter feeder motor disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    flywheelDisconnected.set(!inputs.flywheelConnected);
    feederDisconnected.set(!inputs.feederConnected);
  }

  /** Spin flywheel to tunable target velocity */
  public void spinUp() {
    io.setFlywheelVelocity(targetVelocityRadPerSec.get());
  }

  /** Stop flywheel */
  public void stopFlywheel() {
    io.setFlywheelVelocity(0.0);
  }

  /** Run feeder (open-loop, intentionally) */
  public void feed() {
    io.setFeederVoltage(feederVoltage.get());
  }

  /** Stop feeder */
  public void stopFeeder() {
    io.setFeederVoltage(0.0);
  }

  /** Check if shooter is at speed */
  public boolean atSpeed() {
    return Math.abs(inputs.flywheelVelocityRadPerSec - targetVelocityRadPerSec.get())
        <= velocityToleranceRadPerSec.get();
  }
}
