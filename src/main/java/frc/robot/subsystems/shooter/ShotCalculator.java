// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/** Thank you so much 6328! */
public class ShotCalculator {
  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  /* Single centered shooter (at robot center) */
  public static final Transform2d robotToShooter =
      new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.kZero);

  private static final Translation2d rightTarget =
      AllianceFlipUtil.apply(new Translation2d(1.5, 1.5));

  private static final Translation2d leftTarget =
      rightTarget.plus(new Translation2d(0, (LinesHorizontal.center - rightTarget.getX()) * 2));

  private Rotation2d hoodAngle = Rotation2d.kZero;

  public record ShotParameters(
      boolean isValid,
      Rotation2d requiredRobotYaw,
      Rotation2d hoodAngle,
      double flywheelSpeedRotPerSec) {}

  // Cache parameters
  private ShotParameters latestShot = null;
  @AutoLogOutput @Setter private Goal goal = Goal.HUB;

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

  /** Compatibility overload — preserves old API. */
  public ShotParameters calculateShot() {
    return calculateShot(robotToShooter);
  }

  /** Calculate shot for a given fixed shooter offset */
  public ShotParameters calculateShot(Transform2d robotToShooter) {
    if (latestShot != null) {
      return latestShot;
    }

    // Calculate estimated pose while accounting for phase delay
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

    // Calculate distance from shooter to target
    Translation2d target = AllianceFlipUtil.apply(goal.pose);
    Logger.recordOutput("ShotCalculator/Target", new Pose2d(target, Rotation2d.kZero));

    Pose2d shooterPose = estimatedPose.transformBy(robotToShooter);
    double shooterToTargetDistance = target.getDistance(shooterPose.getTranslation());

    // Calculate field relative shooter velocity
    Rotation2d robotAngle = estimatedPose.getRotation();
    double shooterVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToShooter.getY() * robotAngle.getCos()
                    - robotToShooter.getX() * robotAngle.getSin());
    double shooterVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToShooter.getX() * robotAngle.getCos()
                    - robotToShooter.getY() * robotAngle.getSin());

    // Account for imparted velocity by robot (shooter) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = shooterPose;
    double lookaheadShooterToTargetDistance = shooterToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadShooterToTargetDistance);
      double offsetX = shooterVelocityX * timeOfFlight;
      double offsetY = shooterVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              shooterPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              shooterPose.getRotation());
      lookaheadShooterToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    Rotation2d requiredRobotYaw = target.minus(lookaheadPose.getTranslation()).getAngle();

    hoodAngle = shotHoodAngleMap.get(lookaheadShooterToTargetDistance);
    latestShot =
        new ShotParameters(
            lookaheadShooterToTargetDistance >= minDistance
                && lookaheadShooterToTargetDistance <= maxDistance,
            requiredRobotYaw,
            hoodAngle,
            shotFlywheelSpeedMap.get(lookaheadShooterToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LatestShot", latestShot);
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);

    return latestShot;
  }

  public void clearCache() {
    latestShot = null;
  }

  @RequiredArgsConstructor
  public enum Goal {
    HUB(FieldConstants.hubCenter),
    LEFT(leftTarget),
    RIGHT(rightTarget);
    public final Translation2d pose;
  }

  // This should maybe be refactored. It takes an extra loop cycle to actually change the setpoint.
  public Command setGoalCommand(Goal goal) {
    return Commands.runOnce(() -> this.goal = goal);
  }
}
