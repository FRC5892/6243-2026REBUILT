// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.objectCameraName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.shooter.SnapToTargetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LoggedTalon.TalonFX.NoOppTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.PhoenixTalonFX;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double MOTOR_OVERHEAT_TEMP_C = 80.0;

  private final CANBus rioCAN = new CANBus("rio");
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Climb climb;
  private final Indexer indexer;
  private final Shooter shooter;
  private final LED led;

  // Controller
  private final CommandXboxController m_drivecontroller = new CommandXboxController(0);
  private final CommandXboxController m_codriverController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                objectCameraName,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        climb = new Climb(new PhoenixTalonFX(13, rioCAN, "RightClimb"));
        // TODO: make climb wiht neos

        intake = new Intake(rioCAN);

        indexer = new Indexer(rioCAN);
        led =
            new LED(
                new LEDIOReal(Constants.LEDConstants.PWM_PORT, Constants.LEDConstants.LED_COUNT));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                objectCameraName,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        climb = new Climb(new TalonFXSimpleMotorSim(13, rioCAN, "Climb", 1, 1));

        intake = new Intake(rioCAN);

        indexer = new Indexer(rioCAN);
        led = new LED(new LEDIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        climb = new Climb(new NoOppTalonFX("RightCLimb", 0));

        intake = new Intake(rioCAN);

        indexer = new Indexer(rioCAN);
        led = new LED(new LEDIO() {});
        break;
    }
    shooter = new Shooter(rioCAN);

    configureButtonBindings();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_drivecontroller.getLeftY(),
            () -> -m_drivecontroller.getLeftX(),
            () -> -m_drivecontroller.getRightX()));

    // Driver: Auto align (hold A)
    m_drivecontroller.a().whileTrue(new SnapToTargetCommand(drive, shooter));
    m_drivecontroller.a().onTrue(Commands.runOnce(() -> led.setAutoAlignActive(true)));
    m_drivecontroller.a().onFalse(Commands.runOnce(() -> led.setAutoAlignActive(false)));

    // Driver: Hood stow (X) - on press, move to stow angle if currently below it
    m_drivecontroller.x().onTrue(shooter.getHood().stowCommand());
    m_drivecontroller.x().onTrue(Commands.runOnce(() -> led.setHoodStowed(true)));

    //  DRIVER INTAKE

    // Toggle intake up/down (B)
    m_drivecontroller.b().toggleOnTrue(intake.deploy());
    m_drivecontroller.b().onTrue(Commands.runOnce(led::toggleIntakeDown));

    // Hold to intake IN (left bumper)
    m_drivecontroller.leftBumper().whileTrue(intake.intakeIn());

    // Hold to intake OUT (right bumper)
    m_drivecontroller.rightBumper().whileTrue(intake.intakeOut());

    // Beach alert (Y hold)
    m_drivecontroller.y().onTrue(Commands.runOnce(() -> led.setBeachAlertActive(true)));
    m_drivecontroller.y().onFalse(Commands.runOnce(() -> led.setBeachAlertActive(false)));

    // CODRIVER

    // Climb
    m_codriverController.povUp().whileTrue(frc.robot.commands.climb.ClimbUpCommand.create(climb));

    m_codriverController
        .povDown()
        .whileTrue(frc.robot.commands.climb.ClimbDownCommand.create(climb));

    // indexer unclog
    m_codriverController.rightTrigger().whileTrue(indexer.unclog());

    // Manual hood (left joystick Y)
    shooter
        .getHood()
        .setDefaultCommand(shooter.getHood().manualControl(() -> -m_codriverController.getLeftY()));

    // Auto align (A)
    m_codriverController.a().whileTrue(new SnapToTargetCommand(drive, shooter));
    m_codriverController.a().onTrue(Commands.runOnce(() -> led.setAutoAlignActive(true)));
    m_codriverController.a().onFalse(Commands.runOnce(() -> led.setAutoAlignActive(false)));

    // Intake toggle (left bumper)
    m_codriverController.leftBumper().toggleOnTrue(intake.deploy());
    m_codriverController.leftBumper().onTrue(Commands.runOnce(led::toggleIntakeDown));

    // Beach alert (Y hold)
    m_codriverController.y().onTrue(Commands.runOnce(() -> led.setBeachAlertActive(true)));
    m_codriverController.y().onFalse(Commands.runOnce(() -> led.setBeachAlertActive(false)));

    // Intake IN (X hold)
    m_codriverController.x().whileTrue(intake.intakeIn());

    // Intake OUT (B hold)
    m_codriverController.b().whileTrue(intake.intakeOut());

    // Shoot / unload (right bumper hold)
    m_codriverController
        .rightBumper()
        .whileTrue(frc.robot.commands.shooter.ShootCommand.shoot(shooter, indexer));

    // Keep high-priority motor disconnect alert updated continuously.
    led.setDefaultCommand(
        led.run(
            () -> {
              led.setMotorOverheated(
                  drive.hasOverheatedMotor()
                      || intake.hasOverheatedMotor()
                      || indexer.hasOverheatedMotor()
                      || shooter.hasOverheatedMotor()
                      || climb.hasOverheatedMotor(MOTOR_OVERHEAT_TEMP_C));
              led.setMotorDisconnected(
                  drive.hasDisconnectedMotor()
                      || intake.hasDisconnectedMotor()
                      || indexer.hasDisconnectedMotor()
                      || shooter.hasDisconnectedMotor()
                      || climb.hasDisconnectedMotor());
              led.setHoodStowed(shooter.getHood().getAngle().getDegrees() >= 69.0);
            }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Returns the vision subsystem for mode-based pipeline control. */
  public Vision getVision() {
    return vision;
  }
}
