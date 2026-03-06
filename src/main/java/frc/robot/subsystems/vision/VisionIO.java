package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;

    // Latest AprilTag target angles (used for simple servoing)
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    // AprilTag pose observations
    public PoseObservation[] poseObservations = new PoseObservation[0];

    // Detected AprilTag IDs
    public int[] tagIds = new int[0];

    // PhotonVision targets (optional logging for multiple objects)
    public PhotonTargetObservation[] photonTargets = new PhotonTargetObservation[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /** PhotonVision target record for logging detected objects */
  public static record PhotonTargetObservation(
      double timestamp,
      double yaw, // horizontal angle
      double pitch, // vertical angle
      double area, // target size / confidence
      double bestDistance, // estimated distance to target
      int id // optional ID if you classify objects
      ) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /** Default method to update inputs; implement in your hardware layer */
  public default void updateInputs(VisionIOInputs inputs) {}
}
