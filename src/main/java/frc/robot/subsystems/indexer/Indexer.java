package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class Indexer extends SubsystemBase {

  private final LoggedTalonFX leftMotor;
  private final LoggedTalonFX rightMotor;

  private final DutyCycleOut request = new DutyCycleOut(0);

  private static final double kFeedPower = 1.0;
  private static final double kReversePower = -1.0;
  private static final double kUnjamInPower = -0.5;
  private static final double kUnjamOutPower = 0.5;

  public enum SystemState {
    FEEDING,
    UNJAMMING_IN,
    UNJAMMING_OUT,
    EXHAUSTING,
    IDLE
  }

  public enum WantedState {
    FEED,
    UNJAM,
    EXHAUST,
    IDLE
  }

  private SystemState systemState = SystemState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  public Indexer(LoggedTalonFX leftMotor, LoggedTalonFX rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void setWantedState(WantedState state) {
    wantedState = state;
  }

  private void setPower(double power) {
    leftMotor.setControl(request.withOutput(power));
    rightMotor.setControl(request.withOutput(power));
  }

  @Override
  public void periodic() {
    switch (systemState) {
      case IDLE:
        setPower(0);
        systemState = nextState();
        break;

      case FEEDING:
        setPower(kFeedPower);
        systemState = nextState();
        break;

      case EXHAUSTING:
        setPower(kReversePower);
        systemState = nextState();
        break;

      case UNJAMMING_OUT:
        setPower(kUnjamOutPower);
        systemState = nextState();
        break;

      case UNJAMMING_IN:
        setPower(kUnjamInPower);
        systemState = nextState();
        break;
    }
  }

  private SystemState nextState() {
    return switch (wantedState) {
      case FEED -> SystemState.FEEDING;
      case UNJAM -> SystemState.UNJAMMING_OUT;
      case EXHAUST -> SystemState.EXHAUSTING;
      default -> SystemState.IDLE;
    };
  }

  public void stop() {
    wantedState = WantedState.IDLE;
  }
}
