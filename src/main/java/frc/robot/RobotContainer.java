// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOReal;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Climb climb;
  private final Flywheel leftFlywheel;
  private final Flywheel rightFlywheel;
  private final Hood leftHood;
  private final Hood rightHood;
  private final Intake intake;
  private final RollerSystem rollers;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController m_codriverController = new CommandXboxController(1);

  // Dashboard chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
        );
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(camera0Name, drive::getRotation),
            new VisionIOLimelight(camera1Name, drive::getRotation)
        );

        intake = new Intake(new RollerSystemIOReal(Constants.Intake.motorId));
        rollers = new RollerSystem(new RollerSystemIOReal(Constants.Rollers.motorId));

        leftFlywheel = new Flywheel(new FlywheelIOReal(Constants.Flywheel.leftMotorId));
        rightFlywheel = new Flywheel(new FlywheelIOReal(Constants.Flywheel.rightMotorId));
        leftHood = new Hood(new HoodIOReal(Constants.Hood.leftMotorId));
        rightHood = new Hood(new HoodIOReal(Constants.Hood.rightMotorId));

        climb = new Climb();
        break;

      case SIM:
        drive = new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight)
        );
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
            new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose)
        );

        intake = new Intake(new RollerSystemIOSim());
        rollers = new RollerSystem(new RollerSystemIOSim());

        leftFlywheel = new Flywheel(new FlywheelIOSim());
        rightFlywheel = new Flywheel(new FlywheelIOSim());
        leftHood = new Hood(new HoodIOSim());
        rightHood = new Hood(new HoodIOSim());

        climb = new Climb();
        break;

      default:
        drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        intake = new Intake(new RollerSystemIO() {});
        rollers = new RollerSystem(new RollerSystemIO() {});

        // Empty Flywheel/Hood for test mode
        leftFlywheel = new Flywheel(new FlywheelIO() {
          @Override
          public void updateInputs(FlywheelIO.FlywheelIOInputs inputs) {}
          @Override
          public void applyOutputs(FlywheelIO.FlywheelIOOutputs outputs) {}
        });
        rightFlywheel = new Flywheel(new FlywheelIO() {
          @Override
          public void updateInputs(FlywheelIO.FlywheelIOInputs inputs) {}
          @Override
          public void applyOutputs(FlywheelIO.FlywheelIOOutputs outputs) {}
        });
        leftHood = new Hood(new HoodIO() {});
        rightHood = new Hood(new HoodIO() {});

        climb = new Climb();
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add drive characterizations
    autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()
    ));

    // Lock to 0° when A button is held
    controller.a().whileTrue(DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> Rotation2d.kZero
    ));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro when Y button is pressed
    controller.y().onTrue(Commands.runOnce(
        () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
        drive
    ).ignoringDisable(true));

    // Climb controls
    m_codriverController.rightTrigger().whileTrue(climb.climbUpCommand());
    m_codriverController.leftTrigger().whileTrue(climb.climbDownCommand());

    // Flywheel/Hood independent controls using LoggedTalonFX
    controller.leftBumper().whileTrue(Commands.parallel(
        leftFlywheel.runFixedCommand(5000),   // left flywheel RPM
        leftHood.runFixedCommand(45.0)        // left hood angle in degrees
    ));

    controller.rightBumper().whileTrue(Commands.parallel(
        rightFlywheel.runFixedCommand(5000),  // right flywheel RPM
        rightHood.runFixedCommand(45.0)       // right hood angle in degrees
    ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
