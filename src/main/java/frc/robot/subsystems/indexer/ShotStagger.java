package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.controls.VoltageOut;

public class ShotStagger extends SubsystemBase {

  private final LoggedTalonFX leftGate;
  private final LoggedTalonFX rightGate;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  // =============================
  // TUNABLE VALUES
  // =============================

  // How fast it alternates (seconds between switches)
  private static final LoggedTunableNumber kSwitchPeriod =
      new LoggedTunableNumber("ShotStagger/SwitchPeriodSec", 0.25); // <-- CHANGE THIS to adjust stagger speed

  // Voltage that BLOCKS a path
  private static final LoggedTunableNumber kBlockVoltage =
      new LoggedTunableNumber("ShotStagger/BlockVoltage", 6.0); // <-- CHANGE THIS to adjust blocking strength

  // Voltage that OPENS a path
  private static final LoggedTunableNumber kOpenVoltage =
      new LoggedTunableNumber("ShotStagger/OpenVoltage", -6.0); // <-- CHANGE THIS to adjust opening amount

  private boolean leftBlocking = true;
  private double lastSwitchTime = 0.0;

  public ShotStagger(LoggedTalonFX leftGate, LoggedTalonFX rightGate) {
    this.leftGate = leftGate;
    this.rightGate = rightGate;

    applyGateStates(); // Ensure one side starts blocking
  }

  @Override
  public void periodic() {
    leftGate.periodic();
    rightGate.periodic();
  }

  private void applyGateStates() {
    if (leftBlocking) {
      leftGate.setControl(voltageOut.withOutput(kBlockVoltage.get()));
      rightGate.setControl(voltageOut.withOutput(kOpenVoltage.get()));
    } else {
      leftGate.setControl(voltageOut.withOutput(kOpenVoltage.get()));
      rightGate.setControl(voltageOut.withOutput(kBlockVoltage.get()));
    }
  }

  private void toggle() {
    leftBlocking = !leftBlocking;
    applyGateStates();
  }

  /**
   * Run while shooting.
   * Alternates which gate is blocking at the tunable speed.
   */
  public Command staggerShots() {
    return run(() -> {
      double currentTime = Timer.getFPGATimestamp();

      if (currentTime - lastSwitchTime >= kSwitchPeriod.get()) {
        toggle();
        lastSwitchTime = currentTime;
      }
    });
  }

  /**
   * Stops both gates.
   */
  public Command stop() {
    return runOnce(() -> {
      leftGate.setControl(voltageOut.withOutput(0));
      rightGate.setControl(voltageOut.withOutput(0));
    });
  }
}
