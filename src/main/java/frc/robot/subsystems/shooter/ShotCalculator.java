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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;

    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

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
      double timeOfFlight = timeOfFlightMap.get(lookaheadDistance);

      double offsetX = robotVelocityX * timeOfFlight;
      double offsetY = robotVelocityY * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              estimatedPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              estimatedPose.getRotation());

      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    robotYaw = target.minus(lookaheadPose.getTranslation()).getAngle();

    hoodAngle = shotHoodAngleMap.get(lookaheadDistance);

    latestShot =
        new ShotParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            robotYaw,
            hoodAngle,
            shotFlywheelSpeedMap.get(lookaheadDistance));

    Logger.recordOutput("ShotCalculator/LatestShot", latestShot);
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/Distance", lookaheadDistance);

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
