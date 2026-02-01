// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.GeomUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private double hoodAngle = Double.NaN;
  private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid, double hoodAngle, double hoodVelocity, double flywheelSpeed) {}

  private ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
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
  }

  // Accept pose and velocity as parameters instead of using RobotState
  public ShootingParameters getParameters(
      Pose2d estimatedPose, ChassisSpeeds robotRelativeVelocity) {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance to target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double distanceToTarget = target.getDistance(estimatedPose.getTranslation());

    // Account for robot velocity in time-of-flight
    double timeOfFlight = timeOfFlightMap.get(distanceToTarget);
    double offsetX = robotRelativeVelocity.vxMetersPerSecond * timeOfFlight;
    double offsetY = robotRelativeVelocity.vyMetersPerSecond * timeOfFlight;
    Translation2d lookaheadPosition =
        estimatedPose.getTranslation().plus(new Translation2d(offsetX, offsetY));
    double lookaheadDistance = target.getDistance(lookaheadPosition);

    // Compute hood angle and velocity
    hoodAngle = shotHoodAngleMap.get(lookaheadDistance).getRadians();
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;

    latestParameters =
        new ShootingParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadDistance", lookaheadDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
