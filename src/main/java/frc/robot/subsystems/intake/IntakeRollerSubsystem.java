package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.RollerSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRollerSubsystem extends RollerSubsystem {

  private static final DoubleSupplier kForwardVolts = () -> 8.0;
  private static final DoubleSupplier kReverseVolts = () -> -8.0;

  public IntakeRollerSubsystem(LoggedTalonFX motor) {
    super(motor, kForwardVolts, kReverseVolts);
  }
}
