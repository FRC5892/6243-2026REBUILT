package frc.robot.simulations.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.simulations.FakeRobotState;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.Goal;
import frc.robot.subsystems.shooter.ShotCalculator.ShotParameters;

public class ShotCalcSim {

  private static void printShotForDistance(
      ShotCalculator calc, Translation2d targetPos, double distanceMeters) {

    // Place robot at a known distance straight back from the target.
    Pose2d fakePose =
        new Pose2d(targetPos.getX() - distanceMeters, targetPos.getY(), new Rotation2d());

    FakeRobotState.setPose(fakePose);
    FakeRobotState.setVelocity(0, 0, 0);

    calc.clearCache();
    ShotParameters shot = calc.calculateShot();

    if (!shot.isValid()) {
      System.out.printf("Distance %.2fm -> INVALID SHOT%n", distanceMeters);
      return;
    }

    // ShotCalculator owns the optimization; this reports the chosen hood/RPM pair.
    System.out.printf(
        "Distance %.2fm -> Hood: %.2f deg | Flywheel: %.0f RPM%n",
        distanceMeters, shot.hoodAngle().getDegrees(), shot.flywheelSpeedRPM());
  }

  public static void main(String[] args) {

    ShotCalculator calc = ShotCalculator.getInstance();

    // Fix the goal to HUB and evaluate user-provided distances against it.
    FakeRobotState.setGoal(Goal.HUB);
    Translation2d hubPos = Goal.HUB.pose;

    // If distances are passed on the command line, evaluate those.
    if (args.length > 0) {
      for (String arg : args) {
        try {
          double distanceMeters = Double.parseDouble(arg);
          printShotForDistance(calc, hubPos, distanceMeters);
        } catch (NumberFormatException ex) {
          System.out.printf("Skipping invalid distance input '%s'%n", arg);
        }
      }
      return;
    }

    // Default sweep: 1.0m to 5.5m.
    for (double d = 1.0; d <= 5.5; d += 0.25) {
      printShotForDistance(calc, hubPos, d);
    }
  }
}
