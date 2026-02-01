package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {

  /** Transform from robot center to flywheel/hood assembly */
  public static final Transform3d robotToShooter =
      new Transform3d(
          0.0, // X offset from robot center to shooter (meters)
          0.0, // Y offset
          0.44, // Z height of shooter (meters)
          Rotation3d.kZero // No rotation needed; hood angle handled separately
          );

  /** Optional: camera-to-shooter transform if vision is used for aiming */
  public static final Transform3d cameraToShooter =
      new Transform3d(
          -0.13, // X offset of camera relative to shooter (meters)
          0.0, // Y offset
          0.28, // Z height
          new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0) // pitch offset for vision
          );

  private ShooterConstants() {}
}
