package frc.robot.simulations.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.simulations.FakeRobotState;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.Goal;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;

public class ShotCalcSim {

  public static void main(String[] args) {

    ShotCalculator calc = ShotCalculator.getInstance();

    // Fix the goal to HUB and sweep robot distance from it.
    FakeRobotState.setGoal(Goal.HUB);
    Translation2d hubPos = Goal.HUB.pose;

    for (double d = 1.3; d <= 5.8; d += 0.25) {

      // Place robot d metres along the -X axis from the hub.
      Pose2d fakePose = new Pose2d(hubPos.getX() - d, hubPos.getY(), new Rotation2d());

      FakeRobotState.setPose(fakePose);
      FakeRobotState.setVelocity(0, 0, 0);

      calc.clearCache();

      ShotParameters shot = calc.calculateShot();

      System.out.printf(
          "Distance %.2fm | Hood: %.2f deg | Flywheel: %.0f RPM | Yaw: %.2f deg | Valid: %s%n",
          d,
          shot.hoodAngle().getDegrees(),
          shot.flywheelSpeedRPM(),
          shot.robotYaw().getDegrees(),
          shot.isValid());
    }
  }
}
