package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoggedTalon.TalonFXS.LoggedTalonFX;

public class Intake extends SubsystemBase {
  private final LoggedTalonFX rollerMotor;
  private final LoggedTalonFX slapDownMotor;
  private final LoggedTunableNumber tunedVoltage =
      new LoggedTunableNumber("Intake/RollerVoltage", 2);
  private final LoggedTunableMeasure<MutAngle> outPosition =
      new LoggedTunableMeasure<>("Intake/OutPosition", Rotation.mutable(0.5));
  private final LoggedTunableMeasure<MutAngle> inPosition =
      new LoggedTunableMeasure<>("Intake/InPosition", Rotation.mutable(0));
  private final LoggedTunableMeasure<MutAngle> tolerance =
      new LoggedTunableMeasure<>("Intake/Tolerance", Rotation.mutable(0.05));

  private final VoltageOut voltageOut = new VoltageOut(tunedVoltage.get()).withEnableFOC(true);
  private final MotionMagicTorqueCurrentFOC mmOut = new MotionMagicTorqueCurrentFOC(0);

  public Intake(LoggedTalonFX rollerMotor, LoggedTalonFX slapDownMotor) {
    // Build a blank config for the roller motor
    var rollerConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    this.rollerMotor = rollerMotor.withConfig(rollerConfig);

    // Build the slap down config manually
    var slapDownConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    slapDownConfig.Slot0.kP = 0;
    slapDownConfig.Slot0.kI = 0;
    slapDownConfig.Slot0.kD = 0;
    slapDownConfig.Slot0.kS = 0;
    slapDownConfig.Slot0.kV = 0;
    slapDownConfig.MotionMagic.MotionMagicAcceleration = 2;
    slapDownConfig.MotionMagic.MotionMagicCruiseVelocity = 5;

    this.slapDownMotor = slapDownMotor.withConfig(slapDownConfig).withMMPIDTuning(slapDownConfig);
  }

  public Command intakeCommand() {
    return startEnd(
        () -> rollerMotor.setControl(voltageOut.withOutput(tunedVoltage.get())),
        () -> rollerMotor.setControl(voltageOut.withOutput(0)));
  }

  public Command extendCommand() {
    return startEnd(() -> slapDownMotor.setControl(mmOut.withPosition(outPosition.get())), () -> {})
        .until(
            () ->
                Math.abs(slapDownMotor.getPosition().in(Rotation) - outPosition.get().in(Rotation))
                    < tolerance.get().in(Rotation));
  }

  public Command retractCommand() {
    return startEnd(() -> slapDownMotor.setControl(mmOut.withPosition(inPosition.get())), () -> {})
        .until(
            () ->
                Math.abs(slapDownMotor.getPosition().in(Rotation) - inPosition.get().in(Rotation))
                    < tolerance.get().in(Rotation));
  }

  public Command intakeSequence() {
    return extendCommand().andThen(intakeCommand());
  }

  @Override
  public void periodic() {
    rollerMotor.periodic();
    slapDownMotor.periodic();
  }
}
