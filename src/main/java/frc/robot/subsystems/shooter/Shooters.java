// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooters extends SubsystemBase {
  private final SparkMax shooterMotor =
      new SparkMax(Constants.IdConstants.SHOOTER_ID, MotorType.kBrushed);

  /** Creates a new CoralSubsystem. */
  public Shooters() {}

  public Command shooterCommand() {
    return runEnd(
        () -> {
          shooterMotor.set(Constants.SpeedConstants.SHOOTER_SPEED);
        },
        () -> {
          shooterMotor.stopMotor();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
