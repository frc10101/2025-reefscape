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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  // private final Intake intake = new Intake();

  private final CANdleSystem candle = new CANdleSystem();

  private final Elevator elevator = new Elevator();

  private final ICEE icee = new ICEE();

  private final Arm arm = new Arm();

  // private final NDexter nDexter = new NDexter();

  private final CommandJoystick controller2 = new CommandJoystick(1);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
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
        break;
    }

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
    autoChooser.addOption("blueAuto", drive.getAuto("blue leave"));
    autoChooser.addOption("redAuto", drive.getAuto("blue leave"));

    autoChooser.addOption("Leave Auto", drive.getAuto("blue leave"));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureSwerveCommands();
    Trigger button1 = new Trigger(controller2.button(1)); // output Coral
    Trigger button2 = new Trigger(controller2.button(2)); // intake
    Trigger button3 = new Trigger(controller2.button(3)); // Ndexter Left Faster
    Trigger button4 = new Trigger(controller2.button(4)); // Ndexter Right Faster
    Trigger button8 = new Trigger(controller2.button(8)); // Intake Down
    Trigger button9 = new Trigger(controller2.button(9)); // Intake Up
    Trigger button14 = new Trigger(controller2.button(14)); // L1
    Trigger button15 = new Trigger(controller2.button(15)); // elevtor HP
    Trigger button16 = new Trigger(controller2.button(16)); // elevatorL3
    Trigger POVUp0 = new Trigger(controller2.pov(0)); // arm Up
    Trigger POVUp45 = new Trigger(controller2.pov(45)); // arm Up
    Trigger POVUp315 = new Trigger(controller2.pov(315)); // arm Up
    Trigger POVDown225 = new Trigger(controller2.pov(225)); // arm Down
    Trigger POVDown180 = new Trigger(controller2.pov(180)); // arm Down
    Trigger POVDown135 = new Trigger(controller2.pov(135)); // arm Down

    // button1.whileTrue(
    //     new ConditionalCommand(intake.stop(), intake.StartSpitout(), icee.getLimitSwitch()));
    // button1.whileTrue(
    //     new ConditionalCommand(
    //         nDexter.stop(), nDexter.NDexterSpinSameReverse(), icee.getLimitSwitch()));
    button1.whileTrue(icee.spitOut());

    // button1.onFalse(intake.stop());

    // button2.whileTrue(
    // new ConditionalCommand(intake.stop(), intake.StartIntake(), icee.getLimitSwitch()));
    // button2.whileTrue(
    // new ConditionalCommand(
    // nDexter.stop(), nDexter.NDexterSpinSameForward(), icee.getLimitSwitch()));
    button2.whileTrue(new ConditionalCommand(icee.stop(), icee.Intake(), icee.getLimitSwitch()));
    // button2.onFalse(intake.stop());

    // button3.whileTrue(
    //     new ConditionalCommand(
    //         nDexter.stop(), nDexter.NDexterSpinLeftFasterForward(), icee.getLimitSwitch()));
    // button4.whileTrue(
    //     new ConditionalCommand(
    //         nDexter.stop(), nDexter.NDexterSpinRightFasterForward(), icee.getLimitSwitch()));

    // button8.whileTrue(intake.IntakeDown());
    // button9.whileTrue(intake.IntakeUp());
    button14.whileTrue(elevator.L1());
    button15.whileTrue(elevator.HumanPlayer());
    button16.whileTrue(elevator.L3());
    POVUp0.whileTrue(arm.armUp());
    POVUp45.whileTrue(arm.armUp());
    POVUp315.whileTrue(arm.armUp());
    POVDown225.whileTrue(arm.armDown());
    POVDown180.whileTrue(arm.armDown());
    POVDown135.whileTrue(arm.armDown());

    controller2
        .axisGreaterThan(1, 0.5)
        .whileTrue(
            new ConditionalCommand(elevator.stop(), elevator.raise(), icee.getLimitSwitch()));
    controller2
        .axisLessThan(1, -0.5)
        .whileTrue(
            new ConditionalCommand(elevator.stop(), elevator.lower(), icee.getLimitSwitch()));
    // icee.ICEELimit().whileTrue(nDexter.stop());
    // icee.ICEELimit().whileTrue(nDexter.stop());

    icee.ICEELimit().onTrue(candle.haveCoral());
    icee.ICEELimit().onFalse(candle.noCoral());

    elevator.elevatorLimit().whileTrue(elevator.stop());
  }

  private void configureSwerveCommands() {
    // Default command, normal field-relative drive\
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
                                // new Rotation2d()
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
            // new Rotation2d()
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? new Rotation2d(Math.PI)
                : new Rotation2d()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
