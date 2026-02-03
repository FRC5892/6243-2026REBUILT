// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean tuningMode = false;

  public static final double loopPeriodSecs = 0.02;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** checks that tuning mode is disabled before deploying code */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (tuningMode) {
        System.err.println("Cannot deploy: tuning mode is enabled.");
        System.exit(1);
      }
    }
  }

  public static class IdConstants {
    public static final int CLIMB_ARM_MOTOR_ID = 12;
    public static final int SHOOTER_ID = 17;
  }

  public static class SpeedConstants {
    public static final double CLIMB_ARM_MOTOR_SPEED = 1;
    public static final double CLIMB_BACK_MOTOR_SPEED = -1;
    public static final double SHOOTER_SPEED = -0.6;
  }

  /**
   * Subsystem-specific hardware IDs. These are simple placeholders so callers like RobotContainer
   * can reference motor IDs (e.g. Constants.Flywheel.leftMotorId). Update these values to match
   * your robot's CAN IDs before deploying to hardware.
   */
  public static class Rollers {
    // Example: roller motor CAN ID
    public static final int motorId = 16;
  }

  public static class Flywheel {
    // Use SHOOTER_ID as a sensible default for the left flywheel; adjust as needed.
    public static final int leftMotorId = IdConstants.SHOOTER_ID;
    // Default to SHOOTER_ID + 1 for a second motor (if present).
    public static final int rightMotorId = IdConstants.SHOOTER_ID + 1;
  }

  public static class Hood {
    // Default placeholder CAN IDs for hood motors. Replace with real IDs.
    public static final int leftMotorId = IdConstants.SHOOTER_ID + 2;
    public static final int rightMotorId = IdConstants.SHOOTER_ID + 3;
  }
}
