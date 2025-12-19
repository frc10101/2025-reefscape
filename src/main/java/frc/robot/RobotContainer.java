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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Daisy;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.auto.AlignToReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator = new Elevator();
  private final Daisy daisy = new Daisy();
  private final AlignToReef alignToReef;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS4Controller controller2 = new CommandPS4Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Field
  private final Field2d m_field = new Field2d();

  public RobotContainer() {
    drive = initializeDriveSubsystem();
  
    // PathPlanner Commands
    NamedCommands.registerCommand("L2",elevator.L2());
    NamedCommands.registerCommand("outputSpin",daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
    NamedCommands.registerCommand("stopSpin",daisy.outputSpin(0));
    NamedCommands.registerCommand("HumanPlayer", elevator.HumanPlayer());
    
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    setupAutoOptions();
    SmartDashboard.putData("Field", m_field);
    alignToReef = new AlignToReef(drive, AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

  // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            m_field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            m_field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            m_field.getObject("path").setPoses(poses);
        });

    // Configure button bindings
    configureButtonBindings();
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
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      autoChooser.addOption("Blue Left", drive.getAuto("Blue Left L3"));
      autoChooser.addOption("Blue Middle", drive.getAuto("Blue Middle L3"));
      autoChooser.addOption("Blue Right", drive.getAuto("Blue Right L3"));
    } else {
      autoChooser.addOption("Red Left", drive.getAuto("Red Left L3"));
      autoChooser.addOption("Red Middle", drive.getAuto("Red Middle L3"));
      autoChooser.addOption("Red Right", drive.getAuto("Red Right L3"));
    }
  }

  private void configureButtonBindings() {
    configureSwerveCommands();

    // Controller 2 button bindings
    bindController2Buttons();

    elevator.elevatorLimit().whileTrue(elevator.stop());
  }

  private void bindController2Buttons() {
    Trigger button1 = new Trigger(controller2.button(1)); // L3
    Trigger button2 = new Trigger(controller2.button(2)); // Human Player
    Trigger button3 = new Trigger(controller2.button(3)); // L2
    Trigger button4 = new Trigger(controller2.button(4)); // Elevator Down
    Trigger button5 = new Trigger(controller2.button(5)); // Hold for Intake
    Trigger button6 = new Trigger(controller2.button(6)); // Output Coral

    button1.onTrue(elevator.L3());
    button2.onTrue(elevator.HumanPlayer());
    button3.onTrue(elevator.L2());
    button4.onTrue(elevator.lower());
    button5.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
    button6.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyIn));

  }

  private void configureSwerveCommands() {
    // Default command, normal field-relative drive
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
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                    ? new Rotation2d(Math.PI)
                                    : new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  public void zeroGyro() {
    drive.setPose(
        new Pose2d(
            drive.getPose().getTranslation(),
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? new Rotation2d(Math.PI)
                : new Rotation2d()));
  }
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}