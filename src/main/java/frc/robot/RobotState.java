// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShotCalculator.Goal;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LinesHorizontal;
import frc.robot.util.FieldConstants.LinesVertical;
import lombok.Getter;
import lombok.Setter;

/** Add your docs here. */
public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
  @Getter @Setter private boolean autoGoal = true;
  public Goal updateGoal(){
    if (!autoGoal) {
      return goal;
    }
    if(topTarget.contains(robotPosition.getTranslation())){
      goal = Goal.RIGHT;
    } else if(bottomTarget.contains(robotPosition.getTranslation())){
      goal = Goal.LEFT;
    } else {
      goal = Goal.HUB;
    }
    return goal;
  } 
  @Getter @Setter private Goal goal;
  @Getter @Setter private ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();
  @Getter @Setter private Pose2d robotPosition = new Pose2d();

  private static final Rectangle2d topTarget = new Rectangle2d(
    new Translation2d(LinesVertical.starting, LinesHorizontal.leftTrenchOpenStart), 
    new Translation2d(FieldConstants.fieldWidth, LinesHorizontal.leftTrenchOpenEnd));
  private static Rectangle2d bottomTarget = new Rectangle2d(
    new Translation2d((2 * LinesVertical.center) - LinesVertical.starting, LinesHorizontal.leftTrenchOpenStart),
    new Translation2d(FieldConstants.fieldWidth, LinesHorizontal.leftTrenchOpenEnd));
}
