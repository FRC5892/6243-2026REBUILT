// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);

    /*
     * Chromebook/low-resource-friendly version:
     *
     * The default PhotonCameraSim renders two simulated camera streams using OpenCV:
     *  - raw stream
     *  - processed stream
     * These streams are nice if you want to actually view the simulated images, but
     * they can be expensive on low-power laptops like Chromebooks.
     *
     * The following is a fully-commented-out alternative that disables both streams,
     * but still allows tag detection and pose simulation to work.
     *
     * It's not perfect, but it's much better than without.
     *
     * Uncomment to use this lightweight version:
     *
     * // var cameraProperties = new SimCameraProperties();
     * // cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
     * // cameraSim.enableRawStream(false);        // disables raw image stream
     * // cameraSim.enableProcessedStream(false);  // disables processed image stream
     *
     * Use/Application: ideal for Chromebooks or low-spec machines where rendering
     * the simulated camera images causes lag, but you still need vision data for testing.
     */

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
