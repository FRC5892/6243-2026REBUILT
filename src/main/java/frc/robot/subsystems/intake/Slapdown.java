package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

/** Controls the slap-down actuator on the intake. */
public class Slapdown extends SubsystemBase {

  private final LoggedTalonFX motor;
  private final DutyCycleOut output = new DutyCycleOut(0);

  private static final double EXTEND_OUTPUT = 0.75;  // adjust for real hardware
  private static final double RETRACT_OUTPUT = -0.75;

  public Slapdown(LoggedTalonFX motor) {
    this.motor = motor;
  }

  /** Extend the slap-down actuator. */
  public void extend() {
    motor.setControl(output.withOutput(EXTEND_OUTPUT));
  }

  /** Retract the slap-down actuator. */
  public void retract() {
    motor.setControl(output.withOutput(RETRACT_OUTPUT));
  }

  /** Stop the slap actuator. */
  public void stop() {
    motor.setControl(output.withOutput(0));
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