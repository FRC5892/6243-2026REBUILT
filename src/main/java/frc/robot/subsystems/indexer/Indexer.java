package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.rollersubsystems.FeederRollerSubsystem;
import frc.robot.subsystems.indexer.rollersubsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.RollerSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/** Container for indexer mechanisms (feeder, indexer rollers). */
public class Indexer {
  private static final double MOTOR_OVERHEAT_TEMP_C = 80.0;

  @Getter private final FeederRollerSubsystem feeder;
  @Getter private final IndexerRollerSubsystem indexerRollers;
  // Track whether the indexer is externally enabled (for toggle behavior)
  private volatile boolean enabled = false;

  public Indexer(CANBus bus) {

    switch (Constants.currentMode) {
      case REAL -> {
        // Mechanism CAN ID convention: 20-29 = intake/indexer block
        // Feeder -> 22, Indexer (kicker) -> 21, Slapdown -> 23, Hopper rollers -> 24
        PhoenixTalonFX feederMotor = new PhoenixTalonFX(22, bus, "Feeder");
        feeder = new FeederRollerSubsystem(feederMotor);

        // Primary indexer motor (called "kicker" in Phoenix Tuner) is CAN ID 21.
        PhoenixTalonFX rightIndexer = new PhoenixTalonFX(21, bus, "IndexerRight");
        // No physical left motor; alias to the same instance so subsystem works with one motor.
        LoggedTalonFX leftIndexer = rightIndexer;

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);
      }

      case SIM -> {
        TalonFXSimpleMotorSim feederMotor = new TalonFXSimpleMotorSim(22, bus, "Feeder", 0.001, 1);
        feeder = new FeederRollerSubsystem(feederMotor);

        // SIM mirror of real: indexer primary = 21
        TalonFXSimpleMotorSim rightIndexer =
            new TalonFXSimpleMotorSim(21, bus, "IndexerRight", 0.001, 1);
        TalonFXSimpleMotorSim leftIndexer = rightIndexer;

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);
      }

      default -> {
        NoOppTalonFX feederMotor = new NoOppTalonFX("Feeder", 0);
        feeder = new FeederRollerSubsystem(feederMotor);

        NoOppTalonFX rightIndexer = new NoOppTalonFX("IndexerRight", 0);
        NoOppTalonFX leftIndexer = rightIndexer;

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);
      }
    }
  }

  /** Enable or disable the feeder and indexer rollers. */
  public void setEnabled(boolean enabled) {
    feeder.setEnabled(enabled);
    indexerRollers.setEnabled(enabled);
    this.enabled = enabled;
    Logger.recordOutput("Indexer/Enabled", enabled ? 1.0 : 0.0);
  }

  /** Toggle the enabled state of the indexer. */
  public void toggleEnabled() {
    setEnabled(!this.enabled);
  }

  /** Returns whether the indexer is currently enabled. */
  public boolean isEnabled() {
    return this.enabled;
  }

  /** Runs indexer ONLY when shooter is ready (flywheel speed + hood angle + robot rotation). */
  public Command runWhenShooterReady(Shooter shooter) {
    return Commands.run(
        () -> {
          if (shooter.isReadyToShoot()) {
            feeder.applyDirection(RollerSubsystem.Direction.FORWARD);
            indexerRollers.applyDirection(RollerSubsystem.Direction.FORWARD);
            Logger.recordOutput("Indexer/ShootingActive", 1.0);
          } else {
            feeder.stopMotor();
            indexerRollers.stopMotor();
            Logger.recordOutput("Indexer/ShootingActive", 0.0);
          }
        },
        feeder,
        indexerRollers);
  }

  /** Basic shooting command (no gating). */
  public Command runForShooting() {
    return feeder
        .runRoller(RollerSubsystem.Direction.FORWARD)
        .alongWith(indexerRollers.runRoller(RollerSubsystem.Direction.FORWARD));
  }

  public Command unclog() {
    return feeder
        .runRoller(RollerSubsystem.Direction.REVERSE)
        .alongWith(indexerRollers.runRoller(RollerSubsystem.Direction.REVERSE));
  }

  /** Optional: staged feeding (prevents double-feeding / jams). */
  public Command runStagedFeed() {
    return Commands.sequence(
        indexerRollers.runRoller(RollerSubsystem.Direction.FORWARD).withTimeout(0.15),
        feeder.runRoller(RollerSubsystem.Direction.FORWARD));
  }

  public Command stopAll() {
    return feeder.stop().alongWith(indexerRollers.stop());
  }

  /** Returns true if any indexer motor is disconnected. */
  public boolean hasDisconnectedMotor() {
    return feeder.hasDisconnectedMotor() || indexerRollers.hasDisconnectedMotor();
  }

  /** Returns true if any indexer motor is above the overheat threshold. */
  public boolean hasOverheatedMotor() {
    return feeder.hasOverheatedMotor(MOTOR_OVERHEAT_TEMP_C)
        || indexerRollers.hasOverheatedMotor(MOTOR_OVERHEAT_TEMP_C);
  }
}
