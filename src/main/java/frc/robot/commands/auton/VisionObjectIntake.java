package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Command that uses vision to detect and drive toward game pieces (balls, notes, etc.) while
 * running the intake. Uses clustering to intelligently target groups of objects.
 *
 * <p>Usage: VisionObjectIntake.withIntake(drive, intake, vision)
 */
public class VisionObjectIntake extends Command {
  private final Drive drive;
  private final Intake intake;
  private final Vision vision;
  private final double timeoutSeconds;
  private final Timer timer = new Timer();

  // Safety boundaries (field coordinates in meters)
  private final double minX;
  private final double maxX;
  private final double minY;
  private final double maxY;

  // Cluster parameters
  private final double clusterDistanceThreshold = 0.5; // meters

  // Control parameters
  private static final double YAW_KP = 0.02; // Proportional gain for turning
  private static final double DRIVE_SPEED_MAX = 0.5; // Max forward speed (m/s)
  private static final double APPROACH_DISTANCE = 0.3; // Stop driving forward when closer (m)
  private static final double DISTANCE_KP = 0.5; // Proportional gain for drive speed

  /**
   * Creates a vision-based object intake command with safety boundaries
   *
   * @param drive Drive subsystem
   * @param intake Intake subsystem
   * @param vision Vision subsystem (must have object detection camera configured)
   * @param timeoutSeconds Maximum duration to run
   * @param minX Minimum X coordinate boundary (meters)
   * @param maxX Maximum X coordinate boundary (meters)
   * @param minY Minimum Y coordinate boundary (meters)
   * @param maxY Maximum Y coordinate boundary (meters)
   */
  public VisionObjectIntake(
      Drive drive,
      Intake intake,
      Vision vision,
      double timeoutSeconds,
      double minX,
      double maxX,
      double minY,
      double maxY) {
    this.drive = drive;
    this.intake = intake;
    this.vision = vision;
    this.timeoutSeconds = timeoutSeconds;
    this.minX = minX;
    this.maxX = maxX;
    this.minY = minY;
    this.maxY = maxY;
    addRequirements(drive);
  }

  /**
   * Creates a vision-based object intake command with default neutral zone boundaries
   *
   * @param drive Drive subsystem
   * @param intake Intake subsystem
   * @param vision Vision subsystem (must have object detection camera configured)
   * @param timeoutSeconds Maximum duration to run
   */
  public VisionObjectIntake(Drive drive, Intake intake, Vision vision, double timeoutSeconds) {
    this(drive, intake, vision, timeoutSeconds, 0.0, 4.0, 0.0, 8.0);
  }

  /** Creates command with 10 second timeout */
  public VisionObjectIntake(Drive drive, Intake intake, Vision vision) {
    this(drive, intake, vision, 10.0);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    // Safety: check if within boundaries
    if (robotPose.getX() < minX
        || robotPose.getX() > maxX
        || robotPose.getY() < minY
        || robotPose.getY() > maxY) {
      drive.stop();
      return;
    }

    // Check if object camera is available
    if (!vision.isObjectCameraConnected()) {
      drive.stop();
      return;
    }

    // Get object detection results using PhotonVision 2026 API
    var results = vision.objectCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
      return;
    }

    // Use the most recent result
    var result = results.get(results.size() - 1);
    if (!result.hasTargets()) {
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
      return;
    }

    List<PhotonTrackedTarget> objects = result.getTargets();

    // Group objects into clusters
    List<List<PhotonTrackedTarget>> clusters = clusterObjects(objects);

    // Find best cluster (bigger + closer + centered)
    List<PhotonTrackedTarget> bestCluster = findBestCluster(clusters);

    if (bestCluster != null && !bestCluster.isEmpty()) {
      // Calculate average yaw and minimum distance to cluster
      double avgYaw =
          bestCluster.stream().mapToDouble(PhotonTrackedTarget::getYaw).average().orElse(0);
      // Use PhotonVision convention: bestCameraToTarget is a field, not a method
      double minDistance =
          bestCluster.stream()
              .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
              .min()
              .orElse(0);

      // Calculate turn speed (positive yaw = turn left)
      double turnSpeed = MathUtil.clamp(avgYaw * YAW_KP, -1.0, 1.0);

      // Calculate forward speed based on distance
      double forwardSpeed = 0;
      if (minDistance > APPROACH_DISTANCE) {
        forwardSpeed = MathUtil.clamp(minDistance * DISTANCE_KP, 0, DRIVE_SPEED_MAX);
      }

      // Apply speeds (field-relative)
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              forwardSpeed,
              0,
              turnSpeed * drive.getMaxAngularSpeedRadPerSec(),
              drive.getRotation());
      drive.runVelocity(speeds);
    } else {
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeoutSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
    // Note: Intake control should be handled by separate command composition
  }

  /**
   * Groups objects into clusters based on distance threshold
   *
   * @param objects List of detected objects
   * @return List of clusters, where each cluster is a list of objects
   */
  private List<List<PhotonTrackedTarget>> clusterObjects(List<PhotonTrackedTarget> objects) {
    List<List<PhotonTrackedTarget>> clusters = new ArrayList<>();

    for (PhotonTrackedTarget obj : objects) {
      boolean addedToCluster = false;

      // Try to add to an existing cluster
      for (List<PhotonTrackedTarget> cluster : clusters) {
        for (PhotonTrackedTarget clusterObj : cluster) {
          if (distance(obj, clusterObj) < clusterDistanceThreshold) {
            cluster.add(obj);
            addedToCluster = true;
            break;
          }
        }
        if (addedToCluster) break;
      }

      // Create new cluster if not added
      if (!addedToCluster) {
        List<PhotonTrackedTarget> newCluster = new ArrayList<>();
        newCluster.add(obj);
        clusters.add(newCluster);
      }
    }

    return clusters;
  }

  /**
   * Finds the best cluster to target (larger clusters that are closer and more centered)
   *
   * @param clusters List of object clusters
   * @return The best cluster to target, or null if none
   */
  private List<PhotonTrackedTarget> findBestCluster(List<List<PhotonTrackedTarget>> clusters) {
    List<PhotonTrackedTarget> bestCluster = null;
    double bestScore = Double.MAX_VALUE;

    for (List<PhotonTrackedTarget> cluster : clusters) {
      double avgYaw = cluster.stream().mapToDouble(PhotonTrackedTarget::getYaw).average().orElse(0);
      double avgDistance =
          cluster.stream()
              .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
              .average()
              .orElse(Double.MAX_VALUE);
      double clusterSize = cluster.size();

      // Lower score is better: penalize distance, reward cluster size, penalize off-center
      double score = avgDistance / clusterSize + Math.abs(avgYaw) * 0.1;

      if (score < bestScore) {
        bestScore = score;
        bestCluster = cluster;
      }
    }

    return bestCluster;
  }

  /**
   * Simple distance approximation between two detected objects using yaw difference and distance
   * difference
   *
   * @param a First object
   * @param b Second object
   * @return Approximate distance between objects
   */
  private double distance(PhotonTrackedTarget a, PhotonTrackedTarget b) {
    double dyaw = a.getYaw() - b.getYaw();
    double distA = a.getBestCameraToTarget().getTranslation().getNorm();
    double distB = b.getBestCameraToTarget().getTranslation().getNorm();
    double ddist = distA - distB;
    return Math.hypot(dyaw, ddist);
  }

  /**
   * Creates a complete VisionObjectIntake command that runs the intake while driving toward objects
   *
   * @param drive Drive subsystem
   * @param intake Intake subsystem
   * @param vision Vision subsystem with object detection camera
   * @return Command that drives toward objects while intaking
   */
  public static Command withIntake(Drive drive, Intake intake, Vision vision) {
    return Commands.parallel(new VisionObjectIntake(drive, intake, vision), intake.intakeIn())
        .andThen(intake.retract());
  }

  /**
   * Creates a complete VisionObjectIntake command with custom timeout
   *
   * @param drive Drive subsystem
   * @param intake Intake subsystem
   * @param vision Vision subsystem with object detection camera
   * @param timeoutSeconds Maximum duration
   * @return Command that drives toward objects while intaking
   */
  public static Command withIntake(
      Drive drive, Intake intake, Vision vision, double timeoutSeconds) {
    return Commands.parallel(
            new VisionObjectIntake(drive, intake, vision, timeoutSeconds), intake.intakeIn())
        .andThen(intake.retract());
  }
}
