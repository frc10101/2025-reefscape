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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Daisy;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MapleSimSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  //private final Drive drive;
  private final SwerveDrive drive2;
  //private final SwerveDriveSimulation driveSimulation;
  private final Elevator elevator = new Elevator();
  private final Daisy daisy = new Daisy();
  //private final AlignToReef alignToReef;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS4Controller controller2 = new CommandPS4Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Field
  private final Field2d m_field = new Field2d();

  public RobotContainer() {

    if (Robot.isReal()) {
      this.drive2 = new Drive(
        new GyroIOPigeon2(),
        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
        new ModuleIOTalonFXReal(TunerConstants.BackRight),
        m_field,
        (pose) -> {}); // Real implementation
  }
  else {
      this.drive2 = new MapleSimSwerve(new GyroIOPigeon2(),
      new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
      new ModuleIOTalonFXReal(TunerConstants.FrontRight),
      new ModuleIOTalonFXReal(TunerConstants.BackLeft),
      new ModuleIOTalonFXReal(TunerConstants.BackRight),
      m_field,
      (pose) -> {}
      ); // Simulation implementation
  }
  
    /*switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                m_field,
                (pose) -> {});

        driveSimulation = null;
        break;
      case SIM:
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                m_field,
                driveSimulation::setSimulationWorldPose);
        break;
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                m_field,
                (pose) -> {});
        driveSimulation = null;
        break;
    }
    */

    // PathPlanner Commands
    NamedCommands.registerCommand("L2", elevator.L2());
    NamedCommands.registerCommand(
        "outputSpin", daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
    NamedCommands.registerCommand("stopSpin", daisy.outputSpin(0));
    NamedCommands.registerCommand("HumanPlayer", elevator.HumanPlayer());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    setupAutoOptions();
    SmartDashboard.putData("Field", m_field);
    /*alignToReef =
        new AlignToReef(drive2, AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
*/
    // Configure button bindings
    configureButtonBindings();
  }
  
  private void setupAutoOptions() {
    /*autoChooser.addOption(
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
    autoChooser.addOption("RedLeft", drive.getAuto("RedLeftL3"));
    */
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      autoChooser.addOption("Leave 1", drive2.getAuto("Blue_leave 1"));
      autoChooser.addOption("Leave 2", drive2.getAuto("Blue_leave 2"));
      autoChooser.addOption("Leave 3", drive2.getAuto("Blue_leave 3"));
    } else {
      autoChooser.addOption("Leave 1", drive2.getAuto("Red_leave 1"));
      autoChooser.addOption("Leave 2", drive2.getAuto("Red_leave 2"));
      autoChooser.addOption("Leave 3", drive2.getAuto("Red_leave 3"));
    }
  }

  private void configureButtonBindings() {
    // configureSwerveCommands();

    // Controller 2 button bindings
    bindController2Buttons();

    elevator.elevatorLimit().whileTrue(elevator.stop());
  }

  private void bindController2Buttons() {
    // Trigger button1 = new Trigger(controller2.button(1)); // Output Coral
    // Trigger button2 = new Trigger(controller2.button(2)); // Hold for Intake
    // Trigger button3 = new Trigger(controller2.button(3)); // Elevator Up
    Trigger button4 = new Trigger(controller2.button(4)); // Elevator Down
    // Trigger button5 = new Trigger(controller2.button(5));
    Trigger button6 = new Trigger(controller2.button(1)); // L3
    Trigger button7 = new Trigger(controller2.button(2)); // Human Player
    Trigger button9 = new Trigger(controller2.button(3)); // L2
    Trigger button10 = new Trigger(controller.button(10)); //ResetGyro
    // button1.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyIn));
    // button2.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
    // button3.whileTrue(elevator.raise());
    button4.whileTrue(elevator.lower());
    button6.whileTrue(elevator.L3());
    button7.whileTrue(elevator.HumanPlayer());
    button9.whileTrue(elevator.L2());
   // button10.whileTrue(elevator.L1());

    final Runnable resetGyro =
    Constants.currentMode == Constants.Mode.SIM
        ? () -> drive2.setPose(drive2.getSimulatedDriveTrainPose())
        : () ->
            drive2.setPose(
                new Pose2d(
                    drive2.getPose().getTranslation(),
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? new Rotation2d(Math.PI)
                        : new Rotation2d()));
    button10.onTrue(Commands.runOnce(resetGyro, drive2));
  }

  private void configureSwerveCommands() {
    // Default command, normal field-relative drive
    drive2.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive2,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive2,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive2::stopWithX, drive2));
    
    final Runnable resetGyro =
    Constants.currentMode == Constants.Mode.SIM
        ? () -> drive2.setPose(drive2.getSimulatedDriveTrainPose())
        : () ->
            drive2.setPose(
                new Pose2d(
                    drive2.getPose().getTranslation(),
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? new Rotation2d(Math.PI)
                        : new Rotation2d()));

    controller.b().onTrue(Commands.runOnce(resetGyro, drive2).ignoringDisable(true));
  }

  public void zeroGyro() {
    drive2.setPose(
        new Pose2d(
            drive2.getPose().getTranslation(),
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? new Rotation2d(Math.PI)
                : new Rotation2d()));
     if (Constants.currentMode == Constants.Mode.SIM) {
      drive2.setPose(drive2.getSimulatedDriveTrainPose());
      return;
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive2.setSimulationWorldPose(new Pose2d(6, 6, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

public void updateSimulation() {
  if (Constants.currentMode != Constants.Mode.SIM) return;

  SimulatedArena.getInstance().simulationPeriodic();
  Logger.recordOutput(
      "FieldSimulation/RobotPosition", drive2.getSimulatedDriveTrainPose());
  Logger.recordOutput(
      "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  Logger.recordOutput(
      "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
}
}
