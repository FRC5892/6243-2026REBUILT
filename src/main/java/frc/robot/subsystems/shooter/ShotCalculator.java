// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
// Lookup tables removed; using simple estimators and gravity parameter instead.
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/** Robot-aim version of 6328-style shot calculator */
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

  private Rotation2d robotYaw;
  private Rotation2d hoodAngle = Rotation2d.kZero;

  public record ShotParameters(
      boolean isValid, Rotation2d robotYaw, Rotation2d hoodAngle, double flywheelSpeedRotPerSec) {}

  private ShotParameters latestShot = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double phaseDelay;

  // Gravity (m/s^2) — exposed as a constant parameter per request
  private static final double GRAVITY = 6.18;

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;
    // Lookup tables removed. Use simple estimators driven by distance and gravity.

    AutoLogOutputManager.addObject(getInstance());
  }

  public ShotParameters calculateShot() {
    if (latestShot != null) {
      return latestShot;
    }

    Pose2d estimatedPose = RobotState.getInstance().getRobotPosition();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotRelativeVelocity();

    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());

    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d target = AllianceFlipUtil.apply(RobotState.getInstance().updateGoal().pose);

    Logger.recordOutput("ShotCalculator/Target", new Pose2d(target, Rotation2d.kZero));

    double robotToTargetDistance = target.getDistance(estimatedPose.getTranslation());

    double robotVelocityX = robotVelocity.vxMetersPerSecond;
    double robotVelocityY = robotVelocity.vyMetersPerSecond;

    Pose2d lookaheadPose = estimatedPose;
    double lookaheadDistance = robotToTargetDistance;

    for (int i = 0; i < 20; i++) {
      double timeOfFlight = estimateTimeOfFlight(lookaheadDistance);

      double offsetX = robotVelocityX * timeOfFlight;
      double offsetY = robotVelocityY * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              estimatedPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              estimatedPose.getRotation());

      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    robotYaw = target.minus(lookaheadPose.getTranslation()).getAngle();

    hoodAngle = estimateHoodAngle(lookaheadDistance);

    latestShot =
        new ShotParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            robotYaw,
            hoodAngle,
            estimateFlywheelSpeed(lookaheadDistance));

    Logger.recordOutput("ShotCalculator/LatestShot", latestShot);
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/Distance", lookaheadDistance);

    return latestShot;
  }

  /**
   * Estimate the time of flight for a ball to the target based on distance and gravity. This is a
   * lightweight empirical estimator (previously provided by a table).
   */
  private static double estimateTimeOfFlight(double distanceMeters) {
    // Base time for short shots, slight growth with distance. Scale inversely with sqrt(g)
    double base = 1.05; // seconds at ~2m
    double slope = 0.03; // seconds per meter
    double t = base + slope * (distanceMeters - 2.0);
    t *= Math.sqrt(6.18 / GRAVITY);
    return Math.max(0.5, t);
  }

  /** Estimate hood angle (degrees) for a given distance. This replaces the previous LUT. */
  private static Rotation2d estimateHoodAngle(double distanceMeters) {
    // Linear interpolation between the previously used endpoints to preserve behavior.
    double d0 = 1.34;
    double a0 = 19.0;
    double d1 = 5.60;
    double a1 = 35.0;
    double t = (distanceMeters - d0) / (d1 - d0);
    t = Math.max(0.0, Math.min(1.0, t));
    double deg = a0 + t * (a1 - a0);
    // Keep the angle in a sensible range
    deg = Math.max(10.0, Math.min(80.0, deg));
    return Rotation2d.fromDegrees(deg);
  }

  /** Estimate flywheel speed (rotations per second) for a given distance. Replaces the LUT. */
  private static double estimateFlywheelSpeed(double distanceMeters) {
    double d0 = 1.34;
    double s0 = 210.0;
    double d1 = 5.60;
    double s1 = 290.0;
    double t = (distanceMeters - d0) / (d1 - d0);
    t = Math.max(0.0, Math.min(1.0, t));
    double speed = s0 + t * (s1 - s0);
    return Math.max(0.0, speed);
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
