package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // PhotonVision camera for object detection
  private final PhotonCamera photonCamera =
      new PhotonCamera("CameraName"); // replace with your dashboard name

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /** Returns the X angle to the best target from AprilTag cameras */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /** ---------------- PhotonVision Object Detection Methods ---------------- */

  /** Returns the best detected target from PhotonVision, or null if none */
  public PhotonTrackedTarget getBestTarget() {
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }

  /** Returns the horizontal angle (yaw) to the detected target, 0 if no target */
  public double getTargetYaw() {
    PhotonTrackedTarget target = getBestTarget();
    return (target != null) ? target.getYaw() : 0.0;
  }

  /** Returns distance to target in meters, 0 if no target */
  public double getTargetDistance() {
    PhotonTrackedTarget target = getBestTarget();
    return (target != null) ? target.getBestCameraToTargetDistance() : 0.0;
  }

  /** Returns the PhotonTrackedTarget closest to the robot intake (smallest distance) */
  public PhotonTrackedTarget getClosestTarget() {
    var result = photonCamera.getLatestResult();
    if (!result.hasTargets()) {
      return null;
    }

    PhotonTrackedTarget closest = null;
    double minDistance = Double.MAX_VALUE;

    for (PhotonTrackedTarget target : result.getTargets()) {
      double distance = target.getBestCameraToTargetDistance();
      if (distance < minDistance) {
        minDistance = distance;
        closest = target;
      }
    }
    return closest;
  }

  /** Convenience method for closest target yaw */
  public double getClosestTargetYaw() {
    PhotonTrackedTarget target = getClosestTarget();
    return (target != null) ? target.getYaw() : 0.0;
  }

  /** Convenience method for closest target distance */
  public double getClosestTargetDistance() {
    PhotonTrackedTarget target = getClosestTarget();
    return (target != null) ? target.getBestCameraToTargetDistance() : 0.0;
  }

  /** Returns the PhotonTrackedTarget closest to the intake by combining distance + angle offset */
  public PhotonTrackedTarget getOptimalTargetForIntake() {
    var result = photonCamera.getLatestResult();
    if (!result.hasTargets()) return null;

    PhotonTrackedTarget optimal = null;
    double bestScore = Double.MAX_VALUE;

    for (PhotonTrackedTarget target : result.getTargets()) {
      // Score: combine distance and yaw to prefer balls directly in front of intake
      double score = target.getBestCameraToTargetDistance() + Math.abs(target.getYaw()) * 0.1;
      if (score < bestScore) {
        bestScore = score;
        optimal = target;
      }
    }
    return optimal;
  }

  /** Convenience yaw for optimal target */
  public double getOptimalTargetYaw() {
    PhotonTrackedTarget target = getOptimalTargetForIntake();
    return (target != null) ? target.getYaw() : 0.0;
  }

  /** Convenience distance for optimal target */
  public double getOptimalTargetDistance() {
    PhotonTrackedTarget target = getOptimalTargetForIntake();
    return (target != null) ? target.getBestCameraToTargetDistance() : 0.0;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over AprilTag cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) tagPoses.add(tagPose.get());
      }

      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        robotPoses.add(observation.pose());
        if (rejectPose) robotPosesRejected.add(observation.pose());
        else robotPosesAccepted.add(observation.pose());

        if (rejectPose) continue;

        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
