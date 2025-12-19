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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.AlignToPose;
import frc.robot.commands.auto.AlignToReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Daisy;
import frc.robot.subsystems.Elevator;
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

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick controller2 = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Field
  private final Field2d m_field = new Field2d();

  private final Elevator elevator = new Elevator();

  private final Daisy daisy = new Daisy();

  private final AlignToReef alignToReef;

  public RobotContainer() {
    drive = initializeDriveSubsystem();

    NamedCommands.registerCommand("L2", elevator.L2());
    NamedCommands.registerCommand("Output", daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
    NamedCommands.registerCommand(
        "Align", new AlignToPose(drive, () -> getCurrentPoseFromLimeLight(), true));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    setupAutoOptions();
    SmartDashboard.putData("Field", m_field);
    alignToReef =
        new AlignToReef(drive, AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

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
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      autoChooser.addOption("Leave 1", drive.getAuto("Blue_leave 1"));
      autoChooser.addOption("Leave 2", drive.getAuto("Blue_leave 2"));
      autoChooser.addOption("Leave 3", drive.getAuto("Blue_leave 3"));
    } else {
      autoChooser.addOption("Leave 1", drive.getAuto("Red_leave 1"));
      autoChooser.addOption("Leave 2", drive.getAuto("Red_leave 2"));
      autoChooser.addOption("Leave 3", drive.getAuto("Red_leave 3"));
    }
  }

  private void configureButtonBindings() {
    configureSwerveCommands();
    configureDriver2Commands();
  }

  private void configureDriver2Commands() {
    // Controller 2 button bindings
    Trigger button1 = new Trigger(controller2.button(1)); // Output Coral
    Trigger button2 = new Trigger(controller2.button(2)); // Hold for Intake
    Trigger button3 = new Trigger(controller2.button(3)); // Limelight Data
    Trigger button4 = new Trigger(controller2.button(4)); // Elevator Down
    // Trigger button5 = new Trigger(controller2.button(5));
    Trigger button6 = new Trigger(controller2.button(6)); // L3
    Trigger button7 = new Trigger(controller2.button(7)); // Human Player
    Trigger button9 = new Trigger(controller2.button(9)); // L2
    Trigger button10 = new Trigger(controller2.button(10)); // L1
    Trigger button11 = new Trigger(controller2.button(11)); // Align to Pose
    // Trigger button12 = new Trigger(controller2.button(12));
    // Trigger button13 = new Trigger(controller2.button(13));
    // Trigger button14 = new Trigger(controller2.button(14)); // Release
    // Trigger button15 = new Trigger(controller2.button(15)); // Hang

    button1.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyIn));
    button2.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
    button1.or(button2).onFalse(daisy.outputSpin(0));

    button1.whileFalse(daisy.outputSpin(0));
    elevator.isHP.onTrue(daisy.ActivateBeamBreak());
    elevator.isHP.onFalse(daisy.DeactivateBeamBreak());
    // button3.whileTrue(elevator.raise());
    // button4.whileTrue(elevator.lower());
    button6.onTrue(elevator.L3());
    button7.onTrue(elevator.HumanPlayer());
    button9.onTrue(elevator.L2());
    button10.onTrue(elevator.L1());

    button3.onTrue(drive.reLocalize());
    var map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    button11.onTrue(new AlignToPose(drive, () -> map.getTagPose(18).get().toPose2d(), false));
  }

  private void configureSwerveCommands() {
    Trigger leftReefButton = new Trigger(controller.leftBumper());
    Trigger rightReefButton = new Trigger(controller.rightBumper());
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
    leftReefButton.onTrue(alignToReef.generateCommand(AlignToReef.FieldBranchSide.LEFT));
    rightReefButton.onTrue(alignToReef.generateCommand(AlignToReef.FieldBranchSide.RIGHT));

    // Example: Bind the Y button to drive the robot with specific translation and rotation
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

  public Pose2d getCurrentPoseFromLimeLight() {
    PoseEstimate botPoseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            Constants.LimelightConstants.limelightName);
    if (botPoseEstimate == null) {
      System.out.println("No Estimate");
      return null;
    }
    Pose2d pose = botPoseEstimate.pose;
    if (pose == null) {
      System.out.println("No Pose");
      return null;
    }
    System.out.println("Method Works");
    pose = new Pose2d(pose.getX(), pose.getY(), pose.getRotation().plus(new Rotation2d(Math.PI)));
    return pose;
  }
}
