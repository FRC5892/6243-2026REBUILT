package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.LoggedDIO.HardwareDIO;
import frc.robot.util.LoggedDIO.SimDIO;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXFlywheelSim;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import lombok.Getter;

/** Container for shooting bits. This class will initialize the propper IO interfaces. */
public class Shooter {
  @Getter private final Flywheel flywheel;
  @Getter private final Hood hood;

  public Shooter(CANBus bus) {
    switch (Constants.currentMode) {
      case REAL -> {
        flywheel =
            new Flywheel(
                new PhoenixTalonFX(
                    25,
                    bus,
                    "Flywheel",
                    new PhoenixTalonFollower(26, MotorAlignmentValue.Aligned)));
        hood =
            new Hood(
                new PhoenixTalonFX(27, bus, "Hood"),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
      }
      case SIM -> {
        flywheel =
            new Flywheel(
                new TalonFXFlywheelSim(
                    25,
                    bus,
                    "Flywheel",
                    0.0007567661,
                    1 / 1.25,
                    new PhoenixTalonFollower(26, MotorAlignmentValue.Aligned)));
        hood =
            new Hood(
                new TalonFXSimpleMotorSim(27, bus, "Hood", 0.0017154536, 1.3),
                SimDIO.fromNT("HoodReverse"),
                SimDIO.fromNT("HoodForward"));
      }
      default -> {
        flywheel = new Flywheel(new NoOppTalonFX("Flywheel", 1));
        hood =
            new Hood(
                new NoOppTalonFX("Hood", 0),
                new HardwareDIO("HoodReverse", 1),
                new HardwareDIO("HoodForward", 2));
      }
    }
  }
}