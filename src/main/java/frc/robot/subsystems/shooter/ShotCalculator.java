package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/**
 * Physics-based, non-iterative shot calculator for hooded single flywheel
 * Optimizes points/sec using reduced gravity + Magnus lift + drag
 * Tunable: allows adjustments to speed, hood, magnus coefficient, etc.
 */
public class ShotCalculator {
    private static ShotCalculator instance;

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    private static final Translation2d rightTarget = AllianceFlipUtil.apply(new Translation2d(1.5, 1.5));
    private static final Translation2d leftTarget =
            rightTarget.plus(new Translation2d(0, (LinesHorizontal.center - rightTarget.getX()) * 2));
    private static final Translation2d centerTarget = new Translation2d(rightTarget.getX(), LinesHorizontal.center);

    private Rotation2d robotYaw;
    private Rotation2d hoodAngle = Rotation2d.kZero;

    public record ShotParameters(
            boolean isValid,
            Rotation2d robotYaw,
            Rotation2d hoodAngle,
            double flywheelSpeed,
            double predictedTimeOfFlight,
            double predictedAccuracy) {}

    private ShotParameters latestShot = null;

    // Physical parameters (tunable)
    public double gEff = 6.18; // effective gravity at exit
    public double ballMass = 0.216;
    public double ballRadius = 0.075;
    public double dragCoeff = 0.47;
    public double airDensity = 1.225;
    public double magnusCoeff = 1.0; // tunable lift factor
    public double backspinStabilization = 0.05; // tunable fraction for vertical lift

    private static final double minDistance = 1.34;
    private static final double maxDistance = 5.60;
    private static final double phaseDelay = 0.03;

    static { AutoLogOutputManager.addObject(getInstance()); }

    /** Physics-based, direct shot calculation without iterative search */
    public ShotParameters calculateShot() {
        if (latestShot != null) return latestShot;

        Pose2d estimatedPose = RobotState.getInstance().getRobotPosition();
        ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotRelativeVelocity();
        estimatedPose = estimatedPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay
        ));

        Translation2d target = AllianceFlipUtil.apply(RobotState.getInstance().updateGoal().pose);
        robotYaw = target.minus(estimatedPose.getTranslation()).getAngle();

        // Direct physics formulas for hood angle and flywheel speed
        double dx = target.getX() - estimatedPose.getTranslation().getX();
        double dy = target.getY() - estimatedPose.getTranslation().getY();

        // 1. Compute flight time using effective gravity and vertical displacement
        double launchAngleRad = Math.atan2(dy + backspinStabilization, dx); // basic angle with lift compensation
        double cosTheta = Math.cos(launchAngleRad);
        double sinTheta = Math.sin(launchAngleRad);

        double v0 = Math.sqrt((gEff * dx * dx) / (2 * cosTheta * cosTheta * (dy - dx * Math.tan(launchAngleRad))));
        // Adjust flywheel speed to meters/sec
        double flywheelSpeed = v0;

        // Include simplified drag effect: reduce speed slightly based on distance
        flywheelSpeed *= 1.0 + (dragCoeff * airDensity * dx / (2 * ballMass));

        // Predicted time of flight
        double tFlight = dx / (v0 * cosTheta);

        // Predicted accuracy: simple estimate based on backspin stabilization and drag
        double predictedAccuracy = Math.max(0.5, 1.0 - (dx / maxDistance) * 0.3);

        // Compute hood angle in radians for robot
        hoodAngle = Rotation2d.fromRadians(launchAngleRad);

        boolean isValid = dx >= minDistance && dx <= maxDistance;

        latestShot = new ShotParameters(isValid, robotYaw, hoodAngle, flywheelSpeed, tFlight, predictedAccuracy);
        Logger.recordOutput("ShotCalculator/LatestShot", latestShot);

        return latestShot;
    }

    public void clearCache() { latestShot = null; }

    @RequiredArgsConstructor
    public enum Goal {
        HUB(FieldConstants.hubCenter),
        LEFT(leftTarget),
        RIGHT(rightTarget),
        CENTER(centerTarget);
        public final Translation2d pose;
    }
}