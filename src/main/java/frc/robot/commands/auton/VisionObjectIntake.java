package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveTrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonTrackedTarget;

public class ClusterAwareBallCollector extends CommandBase {
  private final DriveTrain drive;
  private final Intake intake;
  private final Vision vision;
  private final double timeoutSeconds;
  private final Timer timer = new Timer();

  // Neutral zone boundaries
  private final double minX = 0.0;
  private final double maxX = 4.0;
  private final double minY = 0.0;
  private final double maxY = 8.0;

  // Cluster parameters
  private final double clusterDistanceThreshold = 0.5; // meters

  public ClusterAwareBallCollector(
      DriveTrain drive, Intake intake, Vision vision, double timeoutSeconds) {
    this.drive = drive;
    this.intake = intake;
    this.vision = vision;
    this.timeoutSeconds = timeoutSeconds;
    addRequirements(drive, intake);
  }

  public ClusterAwareBallCollector(DriveTrain drive, Intake intake, Vision vision) {
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

    // Safety: stay in neutral zone
    if (robotPose.getX() < minX
        || robotPose.getX() > maxX
        || robotPose.getY() < minY
        || robotPose.getY() > maxY) {
      drive.stop();
      intake.stop();
      return;
    }

    // Get all detected balls
    var result = vision.photonCamera.getLatestResult();
    if (!result.hasTargets()) {
      intake.stop();
      drive.driveForward(0);
      return;
    }
    List<PhotonTrackedTarget> balls = result.getTargets();

    // Group balls into clusters
    List<List<PhotonTrackedTarget>> clusters = new ArrayList<>();
    for (PhotonTrackedTarget ball : balls) {
      boolean added = false;
      for (List<PhotonTrackedTarget> cluster : clusters) {
        for (PhotonTrackedTarget cball : cluster) {
          if (distance(ball, cball) < clusterDistanceThreshold) {
            cluster.add(ball);
            added = true;
            break;
          }
        }
        if (added) break;
      }
      if (!added) {
        List<PhotonTrackedTarget> newCluster = new ArrayList<>();
        newCluster.add(ball);
        clusters.add(newCluster);
      }
    }

    // Score clusters: bigger + closer + aligned with intake
    List<PhotonTrackedTarget> bestCluster = null;
    double bestScore = Double.MAX_VALUE;
    for (List<PhotonTrackedTarget> cluster : clusters) {
      double avgYaw = cluster.stream().mapToDouble(PhotonTrackedTarget::getYaw).average().orElse(0);
      double avgDistance =
          cluster.stream()
              .mapToDouble(PhotonTrackedTarget::getBestCameraToTargetDistance)
              .average()
              .orElse(Double.MAX_VALUE);
      double clusterSize = cluster.size();
      double score = avgDistance / clusterSize + Math.abs(avgYaw) * 0.1; // lower = better
      if (score < bestScore) {
        bestScore = score;
        bestCluster = cluster;
      }
    }

    if (bestCluster != null && !bestCluster.isEmpty()) {
      // Pick central target of cluster
      double yaw =
          bestCluster.stream().mapToDouble(PhotonTrackedTarget::getYaw).average().orElse(0);
      double distance =
          bestCluster.stream()
              .mapToDouble(PhotonTrackedTarget::getBestCameraToTargetDistance)
              .min()
              .orElse(0);

      // Turn toward cluster
      double turnSpeed = yaw * 0.02;
      drive.turn(turnSpeed);

      // Drive forward if far
      if (distance > 0.3) {
        drive.driveForward(Math.min(distance * 0.5, 0.5));
      } else {
        drive.driveForward(0);
      }

      intake.spin(true);
    } else {
      intake.stop();
      drive.driveForward(0);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeoutSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    drive.stop();
    timer.stop();
  }

  /** Simple 3D distance approximation using camera distance + yaw difference */
  private double distance(PhotonTrackedTarget a, PhotonTrackedTarget b) {
    double dyaw = a.getYaw() - b.getYaw();
    double ddist = a.getBestCameraToTargetDistance() - b.getBestCameraToTargetDistance();
    return Math.hypot(dyaw, ddist);
  }
}
