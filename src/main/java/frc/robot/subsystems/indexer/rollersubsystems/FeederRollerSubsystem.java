package frc.robot.subsystems.indexer.rollersubsystems;

import frc.robot.util.RollerSubsystem;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import java.util.function.DoubleSupplier;

public class FeederRollerSubsystem extends RollerSubsystem {

  private static final DoubleSupplier kForwardVolts = () -> 8.0;
  private static final DoubleSupplier kReverseVolts = () -> -8.0;

  public FeederRollerSubsystem(LoggedTalonFX motor) {
    super(motor, kForwardVolts, kReverseVolts);
  }
}
