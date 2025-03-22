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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private final CANdleSystem candle = new CANdleSystem();
  private final Elevator elevator = new Elevator();
  private final ICEE icee = new ICEE();
  private final Arm arm = new Arm();

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick controller2 = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Field
  private final Field2d m_field = new Field2d();

  public RobotContainer() {
    drive = initializeDriveSubsystem();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    setupAutoOptions();
    SmartDashboard.putData("Field", m_field);

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
    autoChooser.addOption("Leave Auto", drive.getAuto("leave"));
  }

  private void configureButtonBindings() {
    configureSwerveCommands();

    // Controller 2 button bindings
    bindController2Buttons();

    // ICEE and CANdle interactions
    icee.ICEELimit().onTrue(candle.haveCoral());
    icee.ICEELimit().onFalse(candle.noCoral());

    elevator.elevatorLimit().whileTrue(elevator.stop());
  }

  private void bindController2Buttons() {
    Trigger button1 = new Trigger(controller2.button(1)); // output Coral
    Trigger button2 = new Trigger(controller2.button(2)); // intake Coral
    Trigger button14 = new Trigger(controller2.button(14)); // L1
    Trigger button15 = new Trigger(controller2.button(15)); // elevator HP
    Trigger button16 = new Trigger(controller2.button(16)); // elevator L3

    button1.whileTrue(icee.spitOut());
    button2.whileTrue(new ConditionalCommand(icee.stop(), icee.Intake(), icee.getLimitSwitch()));
    button14.whileTrue(elevator.L1());
    button15.whileTrue(elevator.HumanPlayer());
    button16.whileTrue(elevator.L3());
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

    icee.ICEELimit().onTrue(arm.coralFF());
    icee.ICEELimit().onFalse(arm.normalFF());
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
