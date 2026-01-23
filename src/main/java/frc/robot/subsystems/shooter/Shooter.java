package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  // Raw inputs (updated each loop) and the AdvantageKit generated loggable class
  private final ShooterIO.ShooterIOInputs inputsRaw = new ShooterIO.ShooterIOInputs();
  private final ShooterIOInputsAutoLogged inputsLogged = new ShooterIOInputsAutoLogged();

  // Tunables
  private final LoggedTunableNumber targetVelocityRadPerSec =
      new LoggedTunableNumber("Shooter/TargetVelocityRadPerSec", 4000.0);
  private final LoggedTunableNumber velocityToleranceRadPerSec =
      new LoggedTunableNumber("Shooter/VelocityToleranceRadPerSec", 20.0);
  private final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("Shooter/FeederVoltage", 6.0);

  // Alerts
  private final Alert flywheelDisconnected =
      new Alert("Shooter flywheel motor disconnected", AlertType.kError);
  private final Alert feederDisconnected =
      new Alert("Shooter feeder motor disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update raw inputs from hardware or simulation first
    io.updateInputs(inputsRaw);

    // Process them through AdvantageKit
    // This will log and (on replay) update inputsLogged based on logged data
    Logger.processInputs("Shooter", inputsLogged);

    // Update alerts based on the logged values
    flywheelDisconnected.set(!inputsLogged.flywheelConnected);
    feederDisconnected.set(!inputsLogged.feederConnected);
  }

  /** Spin flywheel to tunable target velocity */
  public void spinUp() {
    io.setFlywheelVelocity(targetVelocityRadPerSec.get());
  }

  /** Stop flywheel */
  public void stopFlywheel() {
    io.setFlywheelVelocity(0.0);
  }

  /** Run feeder (open‑loop) */
  public void feed() {
    io.setFeederVoltage(feederVoltage.get());
  }

  /** Stop feeder */
  public void stopFeeder() {
    io.setFeederVoltage(0.0);
  }

  // Check if shooter is at speed
  public boolean atSpeed() {
    return Math.abs(inputsLogged.flywheelVelocityRadPerSec - targetVelocityRadPerSec.get())
        <= velocityToleranceRadPerSec.get();
  }
}
