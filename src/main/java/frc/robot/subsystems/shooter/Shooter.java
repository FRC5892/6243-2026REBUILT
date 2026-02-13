<<<<<<< Updated upstream
=======
<<<<<<< HEAD
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Shooter extends SubsystemBase {

  private final ShooterIO io;

  // AdvantageKit-generated inputs container
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Tunables
  private final LoggedTunableNumber targetVelocityRadPerSec =
      new LoggedTunableNumber("Shooter/TargetVelocityRadPerSec", 400.0);

  private final LoggedTunableNumber velocityToleranceRadPerSec =
      new LoggedTunableNumber("Shooter/VelocityToleranceRadPerSec", 20.0);

  private final LoggedTunableNumber feederVoltage =
      new LoggedTunableNumber("Shooter/FeederVoltage", 6.0);

  // Alerts
  private final Alert flywheelDisconnected =
      new Alert("Shooter flywheel motor disconnected", AlertType.kError);

  private final Alert feederDisconnected =
      new Alert("Shooter feeder motor disconnected", AlertType.kError);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Hardware health monitoring
    flywheelDisconnected.set(!inputs.flywheelConnected);
    feederDisconnected.set(!inputs.feederConnected);
  }

  /** Spin flywheel to target velocity (closed-loop handled in IO) */
  public void spinUp() {
    io.setFlywheelVelocity(targetVelocityRadPerSec.get());
    io.setFeederVoltage(0.0);
  }

  /**
   * Fire when at speed
   *
   * @return
   */
  public Command shoot() {
    io.setFlywheelVelocity(targetVelocityRadPerSec.get());
    io.setFeederVoltage(atTargetVelocity() ? feederVoltage.get() : 0.0);
  }

  /** Stop everything */
  public void stop() {
    io.setFlywheelVelocity(0.0);

  /** Check if flywheel is within tolerance */
  public boolean atTargetVelocity() {
    return MathUtil.isNear(
        targetVelocityRadPerSec.get(),
        inputs.flywheelVelocityRadPerSec,
        velocityToleranceRadPerSec.get());
  }

  public Shooter() {
    // TODO Auto-generated constructor stub
  }
}
=======
>>>>>>> Stashed changes
public class Shooter {}
