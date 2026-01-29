// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
  // MARK: - COMPBOT Constants

  public static final double compbotTrackWidthXInches = 20.75;
  public static final double compbotTrackWidthYInches = 20.75;
  public static final double compbotMaxLinearSpeed = 4.69;
  public static final double compbotMaxAngularSpeed = 6.46; // maxLinearSpeed / driveBaseRadius
  public static final double compbotWheelRadiusInches = 2.0;

  // SDS MK5i modules, R2 reduction
  public static final double compbotDriveReduction = 6.02678571429;
  public static final double compbotTurnReduction = 26.0;

  public static final String compbotCanBus = "*";
  public static final int compbotGyroId = 1;
  public static final int compbotDriveMotorIdFL = 2;
  public static final int compbotDriveMotorIdFR = 3;
  public static final int compbotDriveMotorIdBL = 4;
  public static final int compbotDriveMotorIdBR = 5;

  public static final int compbotTurnMotorIdFL = 6;
  public static final int compbotTurnMotorIdFR = 7;
  public static final int compbotTurnMotorIdBL = 8;
  public static final int compbotTurnMotorIdBR = 9;

  public static final int compbotEncoderIdFL = 41;
  public static final int compbotEncoderIdFR = 42;
  public static final int compbotEncoderIdBL = 43;
  public static final int compbotEncoderIdBR = 44;

  public static final double compbotEncoderOffsetFL = 0.0;
  public static final double compbotEncoderOffsetFR = 0.0;
  public static final double compbotEncoderOffsetBL = 0.0;
  public static final double compbotEncoderOffsetBR = 0.0;
}