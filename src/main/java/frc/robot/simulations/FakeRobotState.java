package frc.robot.simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShotCalculator.Goal;

// Injects test state directly into the RobotState singleton for offline simulation.
public class FakeRobotState {

  public static void setPose(Pose2d p) {
    RobotState.getInstance().setRobotPosition(p);
  }

  public static void setVelocity(double vx, double vy, double omega) {
    RobotState.getInstance().setRobotRelativeVelocity(new ChassisSpeeds(vx, vy, omega));
  }

  public static void setGoal(Goal g) {
    RobotState.getInstance().setAutoGoal(false);
    RobotState.getInstance().setGoal(g);
  }
}
