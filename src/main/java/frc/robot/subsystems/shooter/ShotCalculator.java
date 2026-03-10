// Inspired by Mechanical Advantage shot calculator
// http://github.com/Mechanical-Advantage
//TODO: Fix this hot mess because of 2/3 of a gravity
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/** Mechanical Advantage-style shot calculator for 6.18 m/s² planet with drag */
public class ShotCalculator {
  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  private static final Translation2d rightTarget =
      AllianceFlipUtil.apply(new Translation2d(1.5, 1.5));
  private static final Translation2d leftTarget = rightTarget.plus(new Translation2d(0, 2.0));
  private static final Translation2d centerTarget = new Translation2d(rightTarget.getX(), 1.0);

  private Rotation2d robotYaw;
  private Rotation2d hoodAngle = Rotation2d.kZero;

  public record ShotParameters(
      boolean isValid, Rotation2d robotYaw, Rotation2d hoodAngle, double flywheelSpeed) {
    /**
     * Returns the flywheel speed converted to rotations per second using the configured wheel
     * radius.
     */
    public double flywheelSpeedRotPerSec() {
      return flywheelSpeed / (2.0 * Math.PI * flywheelWheelRadiusMeters);
    }

    /** Returns the flywheel speed in RPM. */
    public double flywheelSpeedRPM() {
      return flywheelSpeedRotPerSec() * 60.0;
    }
  }

  private ShotParameters latestShot = null;

  private static final double minDistance = 1.34;
  private static final double maxDistance = 5.60;
  private static final double phaseDelay = 0.03;

  private static final double GRAVITY = 6.18;

  // Drag parameters
  private static final double rho = 1.225;
  private static final double Cd = 0.47;
  private static final double radius = 0.075;
  private static final double area = Math.PI * radius * radius;
  private static final double mass = 0.216;

  // Shooter wheel / flywheel radius (meters). Used to convert a linear exit
  // velocity (m/s) to wheel rotations per second. Tunable if your wheel
  // diameter is different.
  private static final double flywheelWheelRadiusMeters = 0.0508; // 4 in diameter wheel

  public ShotCalculator() {
    AutoLogOutputManager.addObject(this);
  }

  public ShotParameters calculateShot() {
    if (latestShot != null) return latestShot;

    Pose2d estimatedPose = RobotState.getInstance().getRobotPosition();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotRelativeVelocity();
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());

    // Phase delay compensation
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d target = AllianceFlipUtil.apply(RobotState.getInstance().updateGoal().pose);
    Logger.recordOutput("ShotCalculator/Target", new Pose2d(target, Rotation2d.kZero));

    double dx = target.getX() - estimatedPose.getX();
    double dy = target.getY() - estimatedPose.getY();
    double distance = Math.hypot(dx, dy);

    robotYaw = new Rotation2d(dx, dy);

    // Lookahead integration to estimate distance accounting for robot velocity
    Pose2d lookaheadPose = estimatedPose;
    double lookaheadDistance = distance;
    for (int i = 0; i < 20; i++) {
      double tof = estimateTimeOfFlight(lookaheadDistance);

      double offsetX = robotVelocity.vxMetersPerSecond * tof;
      double offsetY = robotVelocity.vyMetersPerSecond * tof;

      lookaheadPose =
          new Pose2d(
              estimatedPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              estimatedPose.getRotation());

      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Estimate optimal hood angle and flywheel speed using drag-aware solver
    double[] result = computeOptimalShot(lookaheadDistance, dy);
    hoodAngle = Rotation2d.fromRadians(result[0]);
    double flywheelSpeed = result[1];

    latestShot =
        new ShotParameters(
            lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance,
            robotYaw,
            hoodAngle,
            flywheelSpeed);

    Logger.recordOutput("ShotCalculator/LatestShot", latestShot);
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/Distance", lookaheadDistance);

    return latestShot;
  }

  /** Estimate time of flight for iterative lookahead (rough) */
  private static double estimateTimeOfFlight(double distanceMeters) {
    // simple approximation, scales with sqrt of gravity
    double base = 1.05;
    double slope = 0.03;
    double t = base + slope * (distanceMeters - 2.0);
    t *= Math.sqrt(9.81 / GRAVITY); // originally calibrated for 9.81
    return Math.max(0.5, t);
  }

  /** Compute hood angle and minimal flywheel speed accounting for drag */
  private double[] computeOptimalShot(double distance, double targetHeight) {
    double vMin = 1.0, vMax = 15.0;
    double tolerance = 0.01;
    double bestV = vMax;
    double bestAngle = Math.atan2(targetHeight, distance); // initial straight-line guess

    for (int iter = 0; iter < 25; iter++) {
      double vTest = (vMin + vMax) / 2.0;
      double angleRad = bestAngle;

      if (simulateShot(distance, targetHeight, vTest, angleRad)) {
        bestV = vTest;
        bestAngle = angleRad;
        vMax = vTest; // try lower speed
      } else {
        vMin = vTest;
      }

      if (vMax - vMin < tolerance) break;
    }

    return new double[] {bestAngle, bestV};
  }

  /** 2D numerical integration with drag and fixed GRAVITY */
  private boolean simulateShot(double dx, double dy, double speed, double angleRad) {
    double dt = 0.005;
    double vx = speed * Math.cos(angleRad);
    double vy = speed * Math.sin(angleRad);
    double y = 0;

    for (int i = 0; i < 2000; i++) {
      double v = Math.hypot(vx, vy);
      double dragAcc = 0.5 * rho * Cd * area * v / mass;

      vx += -dragAcc * (vx / v) * dt;
      vy += (-GRAVITY - dragAcc * (vy / v)) * dt;

      y += vy * dt;

      if (y >= dy) return true;
      if (y < 0) return false;
    }
    return false;
  }

  public void clearCache() {
    latestShot = null;
  }

  @RequiredArgsConstructor
  public enum Goal {
    HUB(centerTarget),
    LEFT(leftTarget),
    RIGHT(rightTarget),
    CENTER(centerTarget);
    public final Translation2d pose;
  }
}
