// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // robot constants

  public static final double TrackWidthXInches = 20.75;
  public static final double TrackWidthYInches = 20.75;
  public static final double MaxLinearSpeed = 4.69;
  public static final double MaxAngularSpeed = 6.46; // maxLinearSpeed / driveBaseRadius
  public static final double WheelRadiusInches = 2.0;

  // SDS MK5i modules, R2 reduction
  public static final double DriveReduction = 6.02678571429;
  public static final double TurnReduction = 26.0;

  // more constants
  public static final String CanBus = "*";
  public static final int GyroId = 1;
  public static final int DriveMotorIdFL = 2;
  public static final int DriveMotorIdFR = 3;
  public static final int DriveMotorIdBL = 4;
  public static final int DriveMotorIdBR = 5;

  public static final int TurnMotorIdFL = 6;
  public static final int TurnMotorIdFR = 7;
  public static final int TurnMotorIdBL = 8;
  public static final int TurnMotorIdBR = 9;

  public static final int EncoderIdFL = 41;
  public static final int EncoderIdFR = 42;
  public static final int EncoderIdBL = 43;
  public static final int EncoderIdBR = 44;

  public static final double EncoderOffsetFL = 0.0;
  public static final double EncoderOffsetFR = 0.0;
  public static final double EncoderOffsetBL = 0.0;
  public static final double EncoderOffsetBR = 0.0;

  public static final double driveKs = 5.0;
  public static final double driveKv = 0.0;
  public static final double driveKp = 35.0;
  public static final double driveKd = 0.0;
  public static final double turnKp = 4000.0;
  public static final double turnKd = 50.0;
  public static final double turnDeadbandDegrees = 0.3;
  public static final double driveCurrentLimitAmps = 80;
  public static final double turnCurrentLimitAmps = 40;

  public static final double trackWidthX = Units.inchesToMeters(TrackWidthXInches);
  public static final double trackWidthY = Units.inchesToMeters(TrackWidthYInches);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = MaxLinearSpeed;
  public static final double maxAngularSpeed = MaxAngularSpeed;
  public static final double wheelRadiusInches = WheelRadiusInches;
  public static final double wheelRadius = Units.inchesToMeters(wheelRadiusInches);
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double driveReduction = DriveReduction;
  public static final double turnReduction = TurnReduction;
}
