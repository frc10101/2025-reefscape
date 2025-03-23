// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ICEE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.Pathfind;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final CANdleSystem candle = new CANdleSystem();
  private final Elevator elevator = new Elevator();
  private final ICEE icee = new ICEE();
  private final Arm arm = new Arm();

  private Field2d field;

  private Command pather = null;

  private Pathfind pathfind;
  // Controller
  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick controller2 = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d m_field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  // private NDexter nDexter = new NDexter();

  // private ICEE icee = new ICEE();

  public RobotContainer() {
    

    try {
      pathfind = new Pathfind();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Field
    drive = initializeDriveSubsystem();
    m_field = new Field2d();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    setupAutoOptions();
    SmartDashboard.putData("Field", m_field);

    // Configure button bindings
    configureButtonBindings();
    setupAutoOptions();
    configureSwerveCommands();
  }

  private Drive initializeDriveSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        return new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight),
            m_field);
      case SIM:
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight),
            m_field);
      default:
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            m_field);
    }
  }

  private void setupAutoOptions() {
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

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    configureSwerveCommands();

    // Controller 2 button bindings
    bindController2Buttons();

    // ICEE and CANdle interactions
    // icee.ICEELimit().onTrue(candle.haveCoral());
    // icee.ICEELimit().onFalse(candle.noCoral());

    // elevator.elevatorLimit().whileTrue(elevator.stop());
  }

  private void bindController2Buttons() {
    // Trigger button1 = new Trigger(controller2.button(1)); // output Coral
    // Trigger button2 = new Trigger(controller2.button(2)); // intake Coral
    // Trigger button14 = new Trigger(controller2.button(14)); // L1
    // Trigger button15 = new Trigger(controller2.button(15)); // elevator HP
    // Trigger button16 = new Trigger(controller2.button(16)); // elevator L3

    // button1.whileTrue(icee.spitOut());
    // button2.whileTrue(new ConditionalCommand(icee.stop(), icee.Intake(), icee.getLimitSwitch()));
    // button14.whileTrue(elevator.L1());
    // button15.whileTrue(elevator.HumanPlayer());
    // button16.whileTrue(elevator.L3());
  }

  private void configureSwerveCommands() {
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // nDexter.setDefaultCommand(nDexter.stop());

    // Trigger x_bot = controller2.button(1);
    // x_bot.whileTrue(nDexter.Out());
    // Trigger circle_bot = controller2.button(2);
    // circle_bot.whileTrue(nDexter.rightFaster());
    // Trigger square_bot = controller2.button(3);
    // square_bot.whileTrue(nDexter.leftFaster());
    // Trigger triangle_bot = controller2.button(4);
    // triangle_bot.whileTrue(nDexter.runSame());

    // Trigger iCeeTriggerIn = controller2.button(5);
    // iCeeTriggerIn.whileTrue(icee.runIn());
    // Trigger iCeeTriggerOut = controller2.button(6);
    // iCeeTriggerOut.whileTrue(icee.runOut());

    // icee.ICEELimit().debounce(.1).onTrue(nDexter.canSpin(false));
    // icee.ICEELimit().debounce(.1).onFalse(nDexter.canSpin(true));

    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  pather = pathfind.pathToPose(Constants.Poses.ReefAPose);
                  pather.schedule();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  pather.cancel();
                }));

    controller.x().onTrue(drive.reLocalize());
  }

  public Command getAutonomousCommand() {
    try {
      // Load the path you want to follow using its name in the GUI
      // PathPlannerPath path = PathPlannerPath.fromPathFile("basic");

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      // return AutoBuilder.followPath(path);
      return new PathPlannerAuto("2Choral");
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
