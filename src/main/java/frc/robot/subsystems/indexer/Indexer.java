package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.rollersubsystems.FeederRollerSubsystem;
import frc.robot.subsystems.indexer.rollersubsystems.IndexerRollerSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.RollerSubsystem;
import lombok.Getter;

/** Container for indexer mechanisms (feeder, indexer rollers). */
public class Indexer {

  @Getter private final FeederRollerSubsystem feeder;
  @Getter private final IndexerRollerSubsystem indexerRollers;

  public Indexer(CANBus bus) {

    switch (Constants.currentMode) {
      case REAL -> {
        PhoenixTalonFX feederMotor = new PhoenixTalonFX(30, bus, "Feeder");
        feeder = new FeederRollerSubsystem(feederMotor);

        PhoenixTalonFX rightIndexer = new PhoenixTalonFX(31, bus, "IndexerRight");

        PhoenixTalonFX leftIndexer =
            new PhoenixTalonFX(
                32, bus, "IndexerLeft", new PhoenixTalonFollower(31, MotorAlignmentValue.Aligned));

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);
      }

      case SIM -> {
        TalonFXSimpleMotorSim feederMotor = new TalonFXSimpleMotorSim(30, bus, "Feeder", 0.001, 1);
        feeder = new FeederRollerSubsystem(feederMotor);

        TalonFXSimpleMotorSim rightIndexer =
            new TalonFXSimpleMotorSim(31, bus, "IndexerRight", 0.001, 1);

        TalonFXSimpleMotorSim leftIndexer =
            new TalonFXSimpleMotorSim(32, bus, "IndexerLeft", 0.001, 1);

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);
      }

      default -> {
        NoOppTalonFX feederMotor = new NoOppTalonFX("Feeder", 0);
        feeder = new FeederRollerSubsystem(feederMotor);

        NoOppTalonFX rightIndexer = new NoOppTalonFX("IndexerRight", 0);
        NoOppTalonFX leftIndexer = new NoOppTalonFX("IndexerLeft", 0);

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);
      }
    }
  }

  /** Runs indexer ONLY when shooter is ready (flywheel speed + hood angle + robot rotation). */
  public Command runWhenShooterReady(Shooter shooter) {
    return Commands.either(
        runForShooting(), stopAll(), shooter::isReadyToShoot // <-- YOU implement this
        );
  }

  /** Basic shooting command (no gating). */
  public Command runForShooting() {
    return feeder
        .runRoller(RollerSubsystem.Direction.FORWARD)
        .alongWith(indexerRollers.runRoller(RollerSubsystem.Direction.FORWARD));
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
}
