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

public class Climb extends SubsystemBase {
  private final LoggedTalonFX climber;

  private final DutyCycleOut request = new DutyCycleOut(0);

  public Climb(LoggedTalonFX climber) {
    this.climber = climber;

    var config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(60));

    this.climber.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), null);
  }

  public void climbUp() {
    climber.setControl(request.withOutput(-0.75));
  }

  public void climbDown() {
    climber.setControl(request.withOutput(0.75));
  }

  public void stopMotor() {
    climber.setControl(request.withOutput(0));
  }

  public Command upCommand() {
    return startEnd(this::climbUp, this::stopMotor);
  }

  public Command downCommand() {
    return startEnd(this::climbDown, this::stopMotor);
  }

  public Command stopAllCommand() {
    return runOnce(
        () -> {
          stopMotor();
        });
  }
}
