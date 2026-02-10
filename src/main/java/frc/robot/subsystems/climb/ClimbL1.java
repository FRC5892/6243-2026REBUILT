package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class ClimbL1 extends SubsystemBase {

  private final LoggedTalonFX climberLeft;
  private final LoggedTalonFX climberRight;

  private final DutyCycleOut leftRequest = new DutyCycleOut(0);
  private final DutyCycleOut rightRequest = new DutyCycleOut(0);

  public ClimbL1(LoggedTalonFX climberLeft, LoggedTalonFX climberRight) {
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;

    var config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(60));

    this.climberLeft.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), null);
    this.climberRight.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), null);
  }

  public void climberLeftUp() {
    climberLeft.setControl(leftRequest.withOutput(-0.75));
  }

  public void climberLeftDown() {
    climberLeft.setControl(leftRequest.withOutput(0.75));
  }

  public void climberRightUp() {
    climberRight.setControl(rightRequest.withOutput(0.75));
  }

  public void climberRightDown() {
    climberRight.setControl(rightRequest.withOutput(-0.75));
  }

  public void stopLeftMotor() {
    climberLeft.setControl(leftRequest.withOutput(0));
  }

  public void stopRightMotor() {
    climberRight.setControl(rightRequest.withOutput(0));
  }

  public Command leftUpCommand() {
    return startEnd(this::climberLeftUp, this::stopLeftMotor);
  }

  public Command leftDownCommand() {
    return startEnd(this::climberLeftDown, this::stopLeftMotor);
  }

  public Command rightUpCommand() {
    return startEnd(this::climberRightUp, this::stopRightMotor);
  }

  public Command rightDownCommand() {
    return startEnd(this::climberRightDown, this::stopRightMotor);
  }

  public Command stopAllCommand() {
    return runOnce(
        () -> {
          stopLeftMotor();
          stopRightMotor();
        });
  }
}
