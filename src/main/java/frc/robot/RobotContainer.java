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
import frc.robot.subsystems.rollers.RollerSystemIOReal;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
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

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController m_codriverController = new CommandXboxController(1);

  // Dashboard chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
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
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        // Intake with placeholder motor ID; replace 0 with actual motor ID later
        intake = new Intake(new RollerSystemIOReal(0)); // TODO: Replace placeholder motor ID

        leftFlywheel = new Flywheel(new FlywheelIOReal(Constants.Flywheel.leftMotorId));
        rightFlywheel = new Flywheel(new FlywheelIOReal(Constants.Flywheel.rightMotorId));

        leftHood = new Hood(new HoodIOReal(Constants.Hood.leftMotorId));
        rightHood = new Hood(new HoodIOReal(Constants.Hood.rightMotorId));

        climb = new Climb();
        break;

      case SIM:
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
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        // Intake with placeholder motor ID for simulation
        intake = new Intake(new RollerSystemIOReal(0)); // TODO: Replace placeholder motor ID

        leftFlywheel = new Flywheel(new FlywheelIOReal(0)); // Sim fallback
        rightFlywheel = new Flywheel(new FlywheelIOReal(0));
        leftHood = new Hood(new HoodIOReal(0));
        rightHood = new Hood(new HoodIOReal(0));

        climb = new Climb();
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        // Intake with placeholder motor ID for fallback mode
        intake = new Intake(new RollerSystemIOReal(0)); // TODO: Replace placeholder motor ID

        leftFlywheel = new Flywheel(new FlywheelIOReal(0));
        rightFlywheel = new Flywheel(new FlywheelIOReal(0));
        leftHood = new Hood(new HoodIOReal(0));
        rightHood = new Hood(new HoodIOReal(0));

        climb = new Climb();
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add drive characterizations
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

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro when Y button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Climb controls
    m_codriverController.rightTrigger().whileTrue(climb.climbUpCommand());
    m_codriverController.leftTrigger().whileTrue(climb.climbDownCommand());

    // Flywheel/Hood independent controls
    controller
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runOnce(() -> leftFlywheel.setVelocity(5000), leftFlywheel),
                Commands.runOnce(() -> leftHood.setPosition(Math.toRadians(45)), leftHood)));

    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runOnce(() -> rightFlywheel.setVelocity(5000), rightFlywheel),
                Commands.runOnce(() -> rightHood.setPosition(Math.toRadians(45)), rightHood)));

    // Intake controls (example: left trigger = intake, right trigger = outtake)
    controller
        .leftTrigger()
        .whileTrue(Commands.run(() -> intake.setGoal(Intake.Goal.INTAKE), intake));
    controller
        .rightTrigger()
        .whileTrue(Commands.run(() -> intake.setGoal(Intake.Goal.OUTTAKE), intake));
    controller.b().onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.STOP), intake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
