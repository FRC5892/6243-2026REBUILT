// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climbArmMotor =
      new SparkMax(Constants.IdConstants.CLIMB_ARM_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  public Command climbUpCommand() {
    return runEnd(
        () -> {
          climbArmMotor.set(Constants.SpeedConstants.CLIMB_ARM_MOTOR_SPEED);
        },
        () -> {
          climbArmMotor.stopMotor();
        });
  }

  public Command climbDownCommand() {
    return runEnd(
        () -> {
          climbArmMotor.set(Constants.SpeedConstants.CLIMB_BACK_MOTOR_SPEED);
        },
        () -> {
          climbArmMotor.stopMotor();
        });
  }

  public Command climbStopCommand() {
    return runEnd(
        () -> {
          climbArmMotor.set(0);
        },
        () -> {
          climbArmMotor.stopMotor();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
