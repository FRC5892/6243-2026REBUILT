// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auton;

// import java.util.ArrayList;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPoint;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.Slapdown;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.vision.Vision;

// /** Add your docs here. */
// public class Paths {

//         public static Command leftSide(Intake intake, Shooter shoot, Intake slapMotor){
//             try {
//       final ArrayList<PathPoint> points;
//       if (Constants.currentMode == Constants.Mode.SIM) {
//         points = new ArrayList<>();
//       } else {
//         points = null;
//       }
//       final Command auto = Commands.sequence(
//               Commands.race(
//                   Commands.sequence(
//                       AutoBuilder.followPath(loadPath("Left_Path_Start", points)),
//                       AutoBuilder.followPath(loadPath("Left_Path_Return", points))),
//                   intake.slapdown(),
//     }
// }
