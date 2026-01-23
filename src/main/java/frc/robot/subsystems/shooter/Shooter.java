package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  // AdvantageKit-generated inputs container
  private final ShooterIOInputsAutoLogged inputs =
      new ShooterIOInputsAutoLogged();

  private final LoggedTunableNumber flywheelVoltage =
      new LoggedTunableNumber("Shooter/FlywheelVoltage", 8.0);

  private final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("Shooter/FeederVoltage", 6.0);

  private final LoggedTunableNumber targetVelocityRadPerSec =
      new LoggedTunableNumber("Shooter/TargetVelocityRadPerSec", 400.0);

  private final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Shooter/VelocityToleranceRadPerSec", 20.0);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void spinUp() {
    io.setFlywheelVoltage(flywheelVoltage.get());
    io.setFeederVoltage(0.0);
  }

  public void shoot() {
    io.setFlywheelVoltage(flywheelVoltage.get());
    io.setFeederVoltage(atTargetVelocity() ? feederVoltage.get() : 0.0);
  }

  public void stop() {
    io.setFlywheelVoltage(0.0);
    io.setFeederVoltage(0.0);
  }

  public boolean atTargetVelocity() {
    return Math.abs(inputs.flywheelVelocityRadPerSec - targetVelocityRadPerSec.get())
        < velocityTolerance.get();
  }
}
