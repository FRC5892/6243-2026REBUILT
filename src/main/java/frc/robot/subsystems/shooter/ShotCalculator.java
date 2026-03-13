package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayDeque;
import java.util.Deque;
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
      boolean isValid, Rotation2d robotYaw, Rotation2d hoodAngle, double flywheelSpeedRPM) {}

  // One entry in the rolling shot history: where the robot was, what goal was targeted, and what
  // parameters were computed. Stored only for valid shots.
  private record RecentShotSample(
      double timestampSec,
      Goal goal,
      Translation2d robotTranslation,
      double distanceMeters,
      Rotation2d hoodAngle,
      double flywheelSpeedRPM) {}

  private ShotParameters latestShot = null;
  // Bounded to ~100 entries (HISTORY_WINDOW / SAMPLE_PERIOD). Each entry is a small record.
  private final Deque<RecentShotSample> recentShotSamples = new ArrayDeque<>();
  private double lastRecentShotSampleSec = Double.NEGATIVE_INFINITY;

  public static boolean manualMode = false;
  public static double manualHoodAngleDeg = 0;
  public static double manualFlywheelSpeed = 0;
  public static double manualRobotYawDeg = 0;

  private static final double minDistance = 1.3;
  private static final double maxDistance = 5.8;
  private static final double phaseDelay = 0.03;
  private static final double validDistanceEpsilon = 1e-6;
  // Keep the last 10 s of shot samples so data is warm when the driver presses shoot.
  private static final double recentShotHistoryWindowSec = 10.0;
  // Save a new sample at most every 100 ms — limits writes to 10 Hz and caps history to ~100
  // entries.
  private static final double recentShotSamplePeriodSec = 0.1;

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

  // Flywheel offset (RPM): raises/lowers all speeds
  // ↑ flywheelOffset → more speed, shots travel farther
  // ↓ flywheelOffset → less speed, shots fall short
  private static final LoggedTunableNumber flywheelOffset =
      new LoggedTunableNumber("ShotTuning/FlywheelOffset", 0.0);

  // Flywheel distance slope (RPM/m): increases/decreases flywheel speed for farther shots
  // ↑ flywheelDistanceSlope → far shots get faster (compensates for distance)
  // ↓ flywheelDistanceSlope → far shots slower (far shots fall short)
  private static final LoggedTunableNumber flywheelDistanceSlope =
      new LoggedTunableNumber("ShotTuning/FlywheelDistanceSlope", 0.0);

  // Time-of-flight scaling factor: adjusts motion compensation
  // ↑ tofScale → robot motion compensation stronger (use if shots overshoot while moving)
  // ↓ tofScale → robot motion compensation weaker (use if shots undershoot while moving)
  private static final LoggedTunableNumber tofScale =
      new LoggedTunableNumber("ShotTuning/TimeOfFlightScale", 1.0);

  // Flywheel RPM at closest shot distance (minDistance)
  private static final LoggedTunableNumber flywheelMinRPM =
      new LoggedTunableNumber("ShotTuning/FlywheelMinRPM", 1000.0);

  // Flywheel RPM at furthest shot distance (maxDistance)
  private static final LoggedTunableNumber flywheelMaxRPM =
      new LoggedTunableNumber("ShotTuning/FlywheelMaxRPM", 5000.0);

  // Biases the curve toward more hood and less RPM while keeping the same distance endpoints.
  // 0.0 = original linear curve, 1.0 = strongest low-RPM preference.
  private static final LoggedTunableNumber lowSpeedPreference =
      new LoggedTunableNumber("ShotTuning/LowSpeedPreference", 0.35);

  // Limits how far from the current pose a saved shot can be and still influence the answer.
  private static final LoggedTunableNumber recentShotPoseBandMeters =
      new LoggedTunableNumber("ShotTuning/RecentShotPoseBandMeters", 0.75);

  // Constant idle RPM the flywheel spins at when not actively shooting
  public static final LoggedTunableNumber flywheelIdleRPM =
      new LoggedTunableNumber("ShotTuning/FlywheelIdleRPM", 3000.0);

  // Hood angle (deg from vertical) at closest shot distance
  private static final LoggedTunableNumber hoodMinAngleDeg =
      new LoggedTunableNumber("ShotTuning/HoodMinAngleDeg", 43.0);

  // Hood angle (deg from vertical) at furthest shot distance
  private static final LoggedTunableNumber hoodMaxAngleDeg =
      new LoggedTunableNumber("ShotTuning/HoodMaxAngleDeg", 68.0);

  static {
    rebuildTables();
    AutoLogOutputManager.addObject(getInstance());
  }

  /** Rebuilds the interpolation tables from the current tunable range values. */
  private static void rebuildTables() {
    hoodAngleMap.clear();
    flywheelSpeedMap.clear();
    timeOfFlightMap.clear();
    double distRange = maxDistance - minDistance;
    double speedPreference = Math.max(0.0, Math.min(1.0, lowSpeedPreference.get()));
    double hoodShapeExponent = 1.0 - (0.4 * speedPreference);
    double flywheelShapeExponent = 1.0 + (0.7 * speedPreference);
    for (double d = minDistance; d <= maxDistance; d += 0.1) {
      double t = (d - minDistance) / distRange;
      // Increase hood a little sooner and delay RPM growth to prefer lower wheel speed.
      double hoodT = Math.pow(t, hoodShapeExponent);
      double flywheelT = Math.pow(t, flywheelShapeExponent);
      double hoodDeg =
          hoodMinAngleDeg.get() + hoodT * (hoodMaxAngleDeg.get() - hoodMinAngleDeg.get());
      double flywheel =
          flywheelMinRPM.get() + flywheelT * (flywheelMaxRPM.get() - flywheelMinRPM.get());
      double tof = 0.82 + (d - 1.3) * 0.085;
      hoodAngleMap.put(d, Rotation2d.fromDegrees(hoodDeg));
      flywheelSpeedMap.put(d, flywheel);
      timeOfFlightMap.put(d, tof);
    }
  }

  public ShotParameters calculateShot() {
    if (latestShot != null) return latestShot;

    // Rebuild tables if any range tunable changed from the dashboard.
    LoggedTunableNumber.ifChanged(
        getInstance(),
        _unused -> {
          rebuildTables();
          getInstance().clearRecentShotHistory();
        },
        flywheelMinRPM,
        flywheelMaxRPM,
        lowSpeedPreference,
        recentShotPoseBandMeters,
        hoodMinAngleDeg,
        hoodMaxAngleDeg);

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

    Goal currentGoal = RobotState.getInstance().updateGoal();
    Translation2d target = AllianceFlipUtil.apply(currentGoal.pose);
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

    // Clamp distance for lookup tables, but keep validity based on the real lookahead distance.
    double distanceClamped = Math.max(minDistance, Math.min(maxDistance, lookaheadDistance));
    double nowSec = Timer.getFPGATimestamp();
    trimRecentShotHistory(nowSec);

    Rotation2d robotYaw = target.minus(lookaheadPose.getTranslation()).getAngle();

    // Base hood/flywheel values
    double hoodDeg = hoodAngleMap.get(distanceClamped).getDegrees();
    double flywheelSpeed = flywheelSpeedMap.get(distanceClamped);

    // Blend in the closest saved sample from the last 10 s.
    // historyWeight is 1.0 when the robot is exactly where it was and fades to 0.0 at
    // poseBandMeters away.
    // This smooths out micro-noise between consecutive loops with almost no CPU cost.
    RecentShotSample recentSample =
        findRecentShotSample(currentGoal, lookaheadPose.getTranslation());
    if (recentSample != null) {
      double poseBandMeters = Math.max(validDistanceEpsilon, recentShotPoseBandMeters.get());
      double poseDelta =
          recentSample.robotTranslation().getDistance(lookaheadPose.getTranslation());
      double historyWeight = 1.0 - Math.min(1.0, poseDelta / poseBandMeters);
      hoodDeg = lerp(hoodDeg, recentSample.hoodAngle().getDegrees(), historyWeight);
      flywheelSpeed = lerp(flywheelSpeed, recentSample.flywheelSpeedRPM(), historyWeight);
      Logger.recordOutput("ShotCalculator/HistoryWeight", historyWeight);
      Logger.recordOutput("ShotCalculator/HistoryPoseDelta", poseDelta);
    } else {
      Logger.recordOutput("ShotCalculator/HistoryWeight", 0.0);
      Logger.recordOutput("ShotCalculator/HistoryPoseDelta", -1.0);
    }

    // Apply tuning offsets
    hoodDeg += hoodOffset.get();
    hoodDeg += hoodDistanceSlope.get() * distanceClamped;

    flywheelSpeed += flywheelOffset.get();
    flywheelSpeed += flywheelDistanceSlope.get() * distanceClamped;

    Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodDeg);

    // Validity must use the real lookahead distance (not the clamped lookup distance).
    boolean isValid =
        lookaheadDistance >= (minDistance - validDistanceEpsilon)
            && lookaheadDistance <= (maxDistance + validDistanceEpsilon);

    latestShot = new ShotParameters(isValid, robotYaw, hoodAngle, flywheelSpeed);

    if (isValid) {
      saveRecentShotSample(
          nowSec,
          currentGoal,
          lookaheadPose.getTranslation(),
          distanceClamped,
          hoodAngle,
          flywheelSpeed);
    }

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

  private void clearRecentShotHistory() {
    recentShotSamples.clear();
    lastRecentShotSampleSec = Double.NEGATIVE_INFINITY;
    Logger.recordOutput("ShotCalculator/RecentHistorySize", 0);
  }

  // Drops entries older than the history window. Called once per loop; at most a handful of
  // removes ever happen at once because saveRecentShotSample throttles the write rate.
  private void trimRecentShotHistory(double nowSec) {
    while (!recentShotSamples.isEmpty()
        && nowSec - recentShotSamples.peekFirst().timestampSec() > recentShotHistoryWindowSec) {
      recentShotSamples.removeFirst();
    }
    Logger.recordOutput("ShotCalculator/RecentHistorySize", recentShotSamples.size());
  }

  // Linear scan over the history deque (≤100 entries) to find the spatially closest sample for
  // the current goal. 100 distance checks per loop is negligible on the roboRIO 2.
  private RecentShotSample findRecentShotSample(Goal goal, Translation2d robotTranslation) {
    double poseBandMeters = Math.max(validDistanceEpsilon, recentShotPoseBandMeters.get());
    RecentShotSample bestSample = null;
    double bestPoseDelta = poseBandMeters;

    for (RecentShotSample sample : recentShotSamples) {
      if (sample.goal() != goal) {
        continue;
      }

      double poseDelta = sample.robotTranslation().getDistance(robotTranslation);
      if (poseDelta <= bestPoseDelta) {
        bestPoseDelta = poseDelta;
        bestSample = sample;
      }
    }

    return bestSample;
  }

  // Saves a new sample at most once per recentShotSamplePeriodSec (100 ms).
  // This prevents alloc churn at 50 Hz and caps the deque at ~100 entries.
  private void saveRecentShotSample(
      double nowSec,
      Goal goal,
      Translation2d robotTranslation,
      double distanceMeters,
      Rotation2d hoodAngle,
      double flywheelSpeedRPM) {
    if (nowSec - lastRecentShotSampleSec < recentShotSamplePeriodSec) {
      return;
    }

    recentShotSamples.addLast(
        new RecentShotSample(
            nowSec, goal, robotTranslation, distanceMeters, hoodAngle, flywheelSpeedRPM));
    lastRecentShotSampleSec = nowSec;
    trimRecentShotHistory(nowSec);
  }

  private static double lerp(double start, double end, double weight) {
    return start + ((end - start) * weight);
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
