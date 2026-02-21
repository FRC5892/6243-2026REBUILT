package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.RollerSubsystem; // optional if you want Direction enum for consistency

/** Controls the slap-down actuator on the intake. */
public class Slapdown extends SubsystemBase {

  private final LoggedTalonFX motor;

  private static final double EXTEND_OUTPUT = 0.75;  // adjust for real hardware
  private static final double RETRACT_OUTPUT = -0.75;

  public Slapdown(LoggedTalonFX motor) {
    this.motor = motor;
  }

  /** Extend the slap-down actuator. */
  public void extend() {
    motor.setControl(motor.getDutyCycle().withOutput(EXTEND_OUTPUT));
  }

  /** Retract the slap-down actuator. */
  public void retract() {
    motor.setControl(motor.getDutyCycle().withOutput(RETRACT_OUTPUT));
  }

  /** Stop the slap actuator. */
  public void stop() {
    motor.setControl(motor.getDutyCycle().withOutput(0));
  }

  /** Command to extend the slap actuator while held. */
  public Command extendCommand() {
    return startEnd(this::extend, this::stop);
  }

  /** Command to retract the slap actuator while held. */
  public Command retractCommand() {
    return startEnd(this::retract, this::stop);
  }
}