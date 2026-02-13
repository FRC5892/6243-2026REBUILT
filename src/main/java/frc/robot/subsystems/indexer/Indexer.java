package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.rollersubsystems.FeederRollerSubsystem;
import frc.robot.subsystems.indexer.rollersubsystems.IndexerRollerSubsystem;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedDIO.SimDIO;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.RollerSubsystem;
import lombok.Getter;

/** Container for indexer mechanisms (feeder, rollers, shot stagger). */
public class Indexer {

  @Getter private final FeederRollerSubsystem feeder;
  @Getter private final IndexerRollerSubsystem indexerRollers;
  @Getter private final ShotStagger shotStagger;

  public Indexer(CANBus bus) {

    switch (Constants.currentMode) {
      case REAL -> {
        // Feeder (single motor)
        PhoenixTalonFX feederMotor = new PhoenixTalonFX(30, bus, "Feeder");

        feeder = new FeederRollerSubsystem(feederMotor);

        // Indexer rollers (right is primary, left follows)
        PhoenixTalonFX rightIndexer = new PhoenixTalonFX(31, bus, "IndexerRight");

        PhoenixTalonFX leftIndexer =
            new PhoenixTalonFX(
                32, bus, "IndexerLeft", new PhoenixTalonFollower(31, MotorAlignmentValue.Aligned));

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);

        // Shot stagger gates
        shotStagger =
            new ShotStagger(new HardwareDIO("ShotPathA", 3), new HardwareDIO("ShotPathB", 4));
      }

      case SIM -> {
        TalonFXSimpleMotorSim feederMotor = new TalonFXSimpleMotorSim(30, bus, "Feeder", 0.001, 1);

        feeder = new FeederRollerSubsystem(feederMotor);

        TalonFXSimpleMotorSim rightIndexer =
            new TalonFXSimpleMotorSim(31, bus, "IndexerRight", 0.001, 1);

        TalonFXSimpleMotorSim leftIndexer =
            new TalonFXSimpleMotorSim(32, bus, "IndexerLeft", 0.001, 1);

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);

        shotStagger = new ShotStagger(SimDIO.fromNT("ShotPathA"), SimDIO.fromNT("ShotPathB"));
      }

      default -> {
        NoOppTalonFX feederMotor = new NoOppTalonFX("Feeder", 0);

        feeder = new FeederRollerSubsystem(feederMotor);

        NoOppTalonFX rightIndexer = new NoOppTalonFX("IndexerRight", 0);

        NoOppTalonFX leftIndexer = new NoOppTalonFX("IndexerLeft", 0);

        indexerRollers = new IndexerRollerSubsystem(rightIndexer, leftIndexer);

        shotStagger =
            new ShotStagger(new HardwareDIO("ShotPathA", 3), new HardwareDIO("ShotPathB", 4));
      }
    }
  }

  /**
   * This should be scheduled whenever the shooter is actively firing. It runs feeder + rollers and
   * lets the shot stagger alternate paths.
   */
  public Command runForShooting() {
    return feeder
        .runRoller(RollerSubsystem.Direction.FORWARD)
        .alongWith(indexerRollers.runRoller(RollerSubsystem.Direction.FORWARD))
        .alongWith(shotStagger.staggerShots());
  }

  public Command stopAll() {
    return feeder.stop().alongWith(indexerRollers.stop()).alongWith(shotStagger.stop());
  }
}
