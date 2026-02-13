package frc.robot.subsystems.indexer.rollersubsystems;

import frc.robot.util.RollerSubsystem;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import java.util.function.DoubleSupplier;

public class IndexerRollerSubsystem extends RollerSubsystem {

  private static final DoubleSupplier kForwardVolts = () -> 8.0;
  private static final DoubleSupplier kReverseVolts = () -> -8.0;

  /**
   * @param leaderMotor The motor that will be directly controlled.
   * @param followerMotor The motor that should already be configured to follow the leader.
   */
  public IndexerRollerSubsystem(
      LoggedTalonFX leaderMotor,
      LoggedTalonFX followerMotor) {

    super(leaderMotor, kForwardVolts, kReverseVolts);

    // We don't command followerMotor directly.
    // It must already be configured as a follower in the container.
  }
}
