package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {

  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  private static final Translation2d rightTarget =
      AllianceFlipUtil.apply(new Translation2d(1.5, 1.5));
  private static final Translation2d leftTarget =
      rightTarget.plus(new Translation2d(0, (LinesHorizontal.center - rightTarget.getX()) * 2));
  private static final Translation2d centerTarget =
      new Translation2d(rightTarget.getX(), LinesHorizontal.center);

  public record ShotParameters(
      boolean isValid, Rotation2d robotYaw, Rotation2d hoodAngle, double flywheelSpeedRotPerSec) {}

  private ShotParameters latestShot = null;

  public static boolean manualMode = false;
  public static double manualHoodAngleDeg = 0;
  public static double manualFlywheelSpeed = 0;
  public static double manualRobotYawDeg = 0;

  private static final double minDistance = 1.3;
  private static final double maxDistance = 5.8;
  private static final double phaseDelay = 0.03;

  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  /*
   * Tunable parameters for real-time adjustment
   * Comments explain what changing each one does
   */

  // Hood offset (degrees): raises/lower all shots equally
  // ↑ hoodOffset → more arc (higher shots, longer distance)
  // ↓ hoodOffset → flatter shots (shorter distance)
  private static final LoggedTunableNumber hoodOffset =
      new LoggedTunableNumber("ShotTuning/HoodOffsetDeg", 0.0);

  // Hood distance slope (degrees per meter): increases/decreases hood angle for farther shots
  // ↑ hoodDistanceSlope → far shots get higher arc
  // ↓ hoodDistanceSlope → far shots get flatter
  private static final LoggedTunableNumber hoodDistanceSlope =
      new LoggedTunableNumber("ShotTuning/HoodDistanceSlope", 0.0);

  // Flywheel offset (RPM or rotations/sec): raises/lowers all speeds
  // ↑ flywheelOffset → more speed, shots travel farther
  // ↓ flywheelOffset → less speed, shots fall short
  private static final LoggedTunableNumber flywheelOffset =
      new LoggedTunableNumber("ShotTuning/FlywheelOffset", 0.0);

  // Flywheel distance slope (RPM/meter): increases/decreases flywheel speed for farther shots
  // ↑ flywheelDistanceSlope → far shots get faster (compensates for distance)
  // ↓ flywheelDistanceSlope → far shots slower (far shots fall short)
  private static final LoggedTunableNumber flywheelDistanceSlope =
      new LoggedTunableNumber("ShotTuning/FlywheelDistanceSlope", 0.0);

  // Time-of-flight scaling factor: adjusts motion compensation
  // ↑ tofScale → robot motion compensation stronger (use if shots overshoot while moving)
  // ↓ tofScale → robot motion compensation weaker (use if shots undershoot while moving)
  private static final LoggedTunableNumber tofScale =
      new LoggedTunableNumber("ShotTuning/TimeOfFlightScale", 1.0);

  static {
    // Populate baseline lookup tables
    for (double d = minDistance; d <= maxDistance; d += 0.1) {
      double hoodDeg = 17 + (d - 1.3) * 4.2; // baseline linear approximation
      double flywheel = 200 + (d - 1.3) * 18.0; // baseline linear
      double tof = 0.82 + (d - 1.3) * 0.085; // baseline flight time

      hoodAngleMap.put(d, Rotation2d.fromDegrees(hoodDeg));
      flywheelSpeedMap.put(d, flywheel);
      timeOfFlightMap.put(d, tof);
    }

    AutoLogOutputManager.addObject(getInstance());
  }

  public ShotParameters calculateShot() {
    if (latestShot != null) return latestShot;

    if (manualMode) {
      // Manual tuning mode: use dashboard-entered values
      latestShot =
          new ShotParameters(
              true,
              Rotation2d.fromDegrees(manualRobotYawDeg),
              Rotation2d.fromDegrees(manualHoodAngleDeg),
              manualFlywheelSpeed);
      return latestShot;
    }

    Pose2d estimatedPose = RobotState.getInstance().getRobotPosition();
    ChassisSpeeds robotRelVel = RobotState.getInstance().getRobotRelativeVelocity();
    ChassisSpeeds robotVel =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, estimatedPose.getRotation());

    // Project pose forward to account for sensor/control delay
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelVel.vxMetersPerSecond * phaseDelay,
                robotRelVel.vyMetersPerSecond * phaseDelay,
                robotRelVel.omegaRadiansPerSecond * phaseDelay));

    Translation2d target = AllianceFlipUtil.apply(RobotState.getInstance().updateGoal().pose);
    double robotToTargetDistance = target.getDistance(estimatedPose.getTranslation());

    double robotVelX = robotVel.vxMetersPerSecond;
    double robotVelY = robotVel.vyMetersPerSecond;

    Pose2d lookaheadPose = estimatedPose;
    double lookaheadDistance = robotToTargetDistance;

    // Motion-compensation loop
    for (int i = 0; i < 20; i++) {
      double tof = timeOfFlightMap.get(lookaheadDistance) * tofScale.get();
      double offsetX = robotVelX * tof;
      double offsetY = robotVelY * tof;
      lookaheadPose =
          new Pose2d(
              estimatedPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              estimatedPose.getRotation());
      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Clamp distance to prevent lookup extrapolation errors
    double distanceClamped = Math.max(minDistance, Math.min(maxDistance, lookaheadDistance));

    Rotation2d robotYaw = target.minus(lookaheadPose.getTranslation()).getAngle();

    // Base hood/flywheel values
    double hoodDeg = hoodAngleMap.get(distanceClamped).getDegrees();
    double flywheelSpeed = flywheelSpeedMap.get(distanceClamped);

    // Apply tuning offsets
    hoodDeg += hoodOffset.get();
    hoodDeg += hoodDistanceSlope.get() * distanceClamped;

    flywheelSpeed += flywheelOffset.get();
    flywheelSpeed += flywheelDistanceSlope.get() * distanceClamped;

    Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodDeg);

    latestShot =
        new ShotParameters(
            distanceClamped >= minDistance && distanceClamped <= maxDistance,
            robotYaw,
            hoodAngle,
            flywheelSpeed);

    // Logging for dashboard tuning
    Logger.recordOutput("ShotCalculator/Distance", distanceClamped);
    Logger.recordOutput(
        "ShotCalculator/HoodAngleBase", hoodAngleMap.get(distanceClamped).getDegrees());
    Logger.recordOutput("ShotCalculator/HoodAngleFinal", hoodDeg);
    Logger.recordOutput("ShotCalculator/FlywheelBase", flywheelSpeedMap.get(distanceClamped));
    Logger.recordOutput("ShotCalculator/FlywheelFinal", flywheelSpeed);

    return latestShot;
  }

  public void clearCache() {
    latestShot = null;
  }

  @RequiredArgsConstructor
  public enum Goal {
    HUB(FieldConstants.hubCenter),
    LEFT(leftTarget),
    RIGHT(rightTarget),
    CENTER(centerTarget);

    public final Translation2d pose;
  }
}
