package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import org.littletonrobotics.junction.Logger;

public class ClimbL3 extends SubsystemBase {

  private final LoggedTalonFX climbLeft;
  private final LoggedTalonFX climbRight;

  private final VoltageOut leftVoltageRequest = new VoltageOut(0);

  private final ArmFeedforward armFF;
  private double lastFF = 0;

  private final DoubleSolenoid extensionSolenoid;

  // FIX #1: constructor name must match class name
  public ClimbL3(LoggedTalonFX climbLeft, LoggedTalonFX climbRight) {
    this.climbLeft = climbLeft;
    this.climbRight = climbRight;

    armFF = new ArmFeedforward(0, 0, 0, 0);

    extensionSolenoid =
        new DoubleSolenoid(8, PneumaticsModuleType.CTREPCM, 0, 2);

    var config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0.5).withKI(0).withKD(0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(60));

    this.climbLeft.withConfig(config).withMMPIDTuning(SlotConfigs.from(config.Slot0), null);

    var followerConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(60));

    this.climbRight.withConfig(followerConfig);
  }

  // Removed Drive dependency since your Drive API doesn’t match
  public Command setVoltageWithFeedforward(double volts, double armAngleRadians, boolean useIdle) {
    return run(
        () -> {
          double ff = armFF.calculate(armAngleRadians, 0);
          lastFF = ff;

          double output = useIdle ? ff : volts + ff;
          climbLeft.setControl(leftVoltageRequest.withOutput(output));
        });
  }

  // FIX #3/4: don’t try to convert Angle → double → back to Angle
  public Command fixPIDPositionReference(double currentAngle) {
    return runOnce(
        () -> {
          lastFF = armFF.calculate(currentAngle, 0);

          // LoggedTalon expects an Angle; reuse the existing position directly
          climbLeft.setPosition(climbLeft.getPosition());
        });
  }

  public Command extend(boolean retract) {
    return runOnce(
        () -> {
          if (retract) {
            extensionSolenoid.set(Value.kReverse);
            Logger.recordOutput("Climber/Solenoid Status", "Retract");
          } else {
            extensionSolenoid.set(Value.kForward);
            Logger.recordOutput("Climber/Solenoid Status", "Extend");
          }
        });
  }

  @Override
  public void periodic() {
    // Your LoggedTalon wrapper already logs internally.
    // Don’t call raw CTRE signal methods that don’t exist here.
    Logger.recordOutput("Climber FeedForward", lastFF);
  }
}
