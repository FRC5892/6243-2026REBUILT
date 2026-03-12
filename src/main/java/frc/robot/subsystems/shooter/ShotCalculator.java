// Inspired by Mechanical Advantage shot calculator
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
   */

  private static final LoggedTunableNumber hoodOffset =
      new LoggedTunableNumber("ShotTuning/HoodOffsetDeg", 0.0);

  private static final LoggedTunableNumber hoodDistanceSlope =
      new LoggedTunableNumber("ShotTuning/HoodDistanceSlope", 0.0);

  private static final LoggedTunableNumber flywheelOffset =
      new LoggedTunableNumber("ShotTuning/FlywheelOffset", 0.0);

  private static final LoggedTunableNumber flywheelDistanceSlope =
      new LoggedTunableNumber("ShotTuning/FlywheelDistanceSlope", 0.0);

  private static final LoggedTunableNumber tofScale =
      new LoggedTunableNumber("ShotTuning/TimeOfFlightScale", 1.0);

  static {
    double d = 1.3;

    // Baseline distance tables; final values are adjusted by tunable offsets/slopes at runtime.
    while (d <= 5.8) {

      double hoodDeg = 17 + (d - 1.3) * 4.2;
      double flywheel = 200 + (d - 1.3) * 18.0;
      double tof = 0.82 + (d - 1.3) * 0.085;

      hoodAngleMap.put(d, Rotation2d.fromDegrees(hoodDeg));
      flywheelSpeedMap.put(d, flywheel);
      timeOfFlightMap.put(d, tof);

      d += 0.1;
    }

    AutoLogOutputManager.addObject(getInstance());
  }

  public ShotParameters calculateShot() {

    // Reuse the per-cycle result until clearCache() is called.
    if (latestShot != null) {
      return latestShot;
    }

    if (manualMode) {

      latestShot =
          new ShotParameters(
              true,
              Rotation2d.fromDegrees(manualRobotYawDeg),
              Rotation2d.fromDegrees(manualHoodAngleDeg),
              manualFlywheelSpeed);

      return latestShot;
    }

    Pose2d estimatedPose = RobotState.getInstance().getRobotPosition();

    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotRelativeVelocity();

    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());

    // Project pose forward slightly to compensate for sensing/actuation delay.
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d target = AllianceFlipUtil.apply(RobotState.getInstance().updateGoal().pose);

    double robotToTargetDistance = target.getDistance(estimatedPose.getTranslation());

    double robotVelocityX = robotVelocity.vxMetersPerSecond;
    double robotVelocityY = robotVelocity.vyMetersPerSecond;

    Pose2d lookaheadPose = estimatedPose;
    double lookaheadDistance = robotToTargetDistance;

    // Solve moving-shot geometry: flight time changes with distance, and distance changes with
    // motion.
    for (int i = 0; i < 20; i++) {

      double timeOfFlight = timeOfFlightMap.get(lookaheadDistance) * tofScale.get();

      double offsetX = robotVelocityX * timeOfFlight;
      double offsetY = robotVelocityY * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              estimatedPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              estimatedPose.getRotation());

      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    Rotation2d robotYaw = target.minus(lookaheadPose.getTranslation()).getAngle();

    double hoodDeg = hoodAngleMap.get(lookaheadDistance).getDegrees();

    hoodDeg += hoodOffset.get();
    hoodDeg += hoodDistanceSlope.get() * lookaheadDistance;

    double flywheelSpeed = flywheelSpeedMap.get(lookaheadDistance);

    flywheelSpeed += flywheelOffset.get();
    flywheelSpeed += flywheelDistanceSlope.get() * lookaheadDistance;

    Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodDeg);

    latestShot =
        new ShotParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            robotYaw,
            hoodAngle,
            flywheelSpeed);

    Logger.recordOutput("ShotCalculator/Distance", lookaheadDistance);
    Logger.recordOutput(
        "ShotCalculator/HoodAngleBase", hoodAngleMap.get(lookaheadDistance).getDegrees());
    Logger.recordOutput("ShotCalculator/HoodAngleFinal", hoodDeg);
    Logger.recordOutput("ShotCalculator/FlywheelBase", flywheelSpeedMap.get(lookaheadDistance));
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
