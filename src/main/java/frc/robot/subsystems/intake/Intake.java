package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import lombok.Getter;

/** Container subsystem for intake rollers + slap-down actuator. */
public class Intake {

  @Getter private final IntakeRollerSubsystem roller;
  @Getter private final Slapdown slap;

  public Intake(CANBus bus) {
    switch (Constants.currentMode) {
      case REAL -> {
        PhoenixTalonFX rollerMotor = new PhoenixTalonFX(20, bus, "IntakeRoller");
        roller = new IntakeRollerSubsystem(rollerMotor);

        PhoenixTalonFX slapMotor = new PhoenixTalonFX(21, bus, "Slapdown");
        slap = new Slapdown(slapMotor);
      }

      case SIM -> {
        TalonFXSimpleMotorSim rollerMotor =
            new TalonFXSimpleMotorSim(20, bus, "IntakeRoller", 0.001, 1);
        roller = new IntakeRollerSubsystem(rollerMotor);

        TalonFXSimpleMotorSim slapMotor = new TalonFXSimpleMotorSim(21, bus, "Slapdown", 0.001, 1);
        slap = new Slapdown(slapMotor);
      }

      default -> {
        NoOppTalonFX rollerMotor = new NoOppTalonFX("IntakeRoller", 0);
        roller = new IntakeRollerSubsystem(rollerMotor);

        NoOppTalonFX slapMotor = new NoOppTalonFX("Slapdown", 0);
        slap = new Slapdown(slapMotor);
      }
    }
  }

  /** Run the intake roller forward. */
  public Command runRollerForward() {
    return roller.runRoller(frc.robot.util.RollerSubsystem.Direction.FORWARD);
  }

  /** Run the intake roller reverse. */
  public Command runRollerReverse() {
    return roller.runRoller(frc.robot.util.RollerSubsystem.Direction.REVERSE);
  }

  /** Stop the intake roller. */
  public Command stopRoller() {
    return roller.stop();
  }

  /** Extend the slap-down actuator. */
  public Command extendSlap() {
    return slap.extendCommand();
  }

  /** Retract the slap-down actuator. */
  public Command retractSlap() {
    return slap.retractCommand();
  }

  /** Deploy intake: rollers forward + slap down. */
  public Command deploy() {
    return Commands.parallel(runRollerForward(), extendSlap());
  }

  /** Retract intake: rollers stopped + slap retracted. */
  public Command retract() {
    return Commands.parallel(stopRoller(), retractSlap());
  }
}
