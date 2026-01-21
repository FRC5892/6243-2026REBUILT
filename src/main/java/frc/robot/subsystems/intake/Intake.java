package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  // Tunable numbers
  private final LoggedTunableNumber stowedAngleRad =
      new LoggedTunableNumber("Intake/StowedAngleRad", 0.0);
  private final LoggedTunableNumber deployedAngleRad =
      new LoggedTunableNumber("Intake/DeployedAngleRad", 1.4);
  private final LoggedTunableNumber armKp = new LoggedTunableNumber("Intake/ArmKp", 6.0);
  private final LoggedTunableNumber maxArmVolts =
      new LoggedTunableNumber("Intake/MaxArmVolts", 8.0);
  private final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/RollerVolts", 6.0);

  private double targetArmAngleRad;
  private boolean rollerRunning = false;

  public Intake(IntakeIO io) {
    this.io = io;
    targetArmAngleRad = stowedAngleRad.get();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

    double error = targetArmAngleRad - inputs.armPositionRad;
    double volts = armKp.get() * error;
    volts = Math.max(-maxArmVolts.get(), Math.min(maxArmVolts.get(), volts));
    io.setArmVoltage(volts);

    io.setRollerVoltage(rollerRunning ? rollerVolts.get() : 0.0);
  }

  public void deploy() {
    targetArmAngleRad = deployedAngleRad.get();
  }

  public void stow() {
    targetArmAngleRad = stowedAngleRad.get();
  }

  public void startRoller() {
    rollerRunning = true;
  }

  public void stopRoller() {
    rollerRunning = false;
  }

  public boolean atTarget() {
    return Math.abs(targetArmAngleRad - inputs.armPositionRad) < 0.05;
  }
}
