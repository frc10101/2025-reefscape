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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

  private Command pather = null;
  private Pathfind pathfind;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d m_field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize field
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    // Initialize drivetrain
    drive = initializeDriveSubsystem();

    // Register commands for PathPlanner
    registerAutoCommands();

    // Set up auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    setupAutoOptions();

    // Initialize pathfinding
    try {
      pathfind = new Pathfind();
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to initialize Pathfind: " + e.getMessage(), e.getStackTrace());
    }

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
    if (Constants.isSysID) {
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
    autoChooser.addOption("redCenter Auto", drive.getAuto("redCenter"));
    autoChooser.addOption("blueCenter Auto", drive.getAuto("blueCenter"));
    autoChooser.addOption("blueAuto", drive.getAuto("blueAutoL4"));
    autoChooser.addOption("redAuto", drive.getAuto("redAutoL4"));
  }

  private void configureButtonBindings() {
    configureSwerveCommands();

    // Controller 2 button bindings
    bindController2Buttons();

    // ICEE and CANdle interactions
    icee.ICEELimit().onTrue(candle.haveCoral());
    icee.ICEELimit().onFalse(candle.noCoral());
  }

  private void bindController2Buttons() {
    Trigger button1 = new Trigger(controller2.button(1)); // output Coral
    Trigger button2 = new Trigger(controller2.button(2)); // intake Coral
    Trigger button5 = new Trigger(controller2.button(5)); // L1
    Trigger button6 = new Trigger(controller2.button(6)); // elevator HP
    Trigger button7 = new Trigger(controller2.button(7)); // elevator L3
    Trigger button8 = new Trigger(controller2.button(8)); // elevator L4
    Trigger button9 = new Trigger(controller2.button(9)); // elevator L2
    Trigger button10 = new Trigger(controller2.button(10)); // elevator L1

    button1.whileTrue(icee.spitOut());
    button2.whileTrue(new ConditionalCommand(icee.stop(), icee.Intake(), icee.getLimitSwitch()));
    button5.onTrue(elevator.L1());
    button6.whileTrue(elevator.HumanPlayer());
    button7.whileTrue(elevator.L3());
    button8.onTrue(elevator.L4());
    button9.whileTrue(elevator.L2());
    button10.whileTrue(elevator.L1());
  }

  private void configureSwerveCommands() {
    // Default command, normal field-relative drive with S-curve motion profile
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> {
              // Apply S-curve profile to Y input (forward/backward)
              double rawY = controller.getLeftY();
              return applySCurveProfile(rawY);
            },
            () -> {
              // Apply S-curve profile to X input (strafe)
              double rawX = controller.getLeftX();
              return applySCurveProfile(rawX);
            },
            () -> {
              // Apply S-curve profile to rotation input
              double rawRotation = controller.getRightX();
              return -1 * applySCurveProfile(rawRotation);
            }));

    // Lock to 0° when A button is held (also with S-curve profile)
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> {
                  double rawY = controller.getLeftY();
                  return applySCurveProfile(rawY);
                },
                () -> {
                  double rawX = controller.getLeftX();
                  return applySCurveProfile(rawX);
                },
                () -> new Rotation2d()));

    // Controller B button for pathfinding to reef A pose (if defined in Constants.Poses)
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (pathfind != null && Constants.Poses.ReefAPose != null) {
                    pather = pathfind.pathToPose(Constants.Poses.ReefAPose);
                    if (pather != null) {
                      pather.schedule();
                    }
                  }
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (pather != null) {
                    pather.cancel();
                  }
                }));

    controller.y().onTrue(drive.reLocalize());
  }

  /**
   * Applies an S-curve motion profile to controller inputs using the function 4x³-3x⁴. This creates
   * smoother acceleration and deceleration with a unique response curve.
   *
   * @param input Raw controller input (-1.0 to 1.0)
   * @return Processed input with S-curve applied
   */
  private double applySCurveProfile(double input) {
    // Apply deadband to prevent drift
    final double deadband = 0.05;
    if (Math.abs(input) < deadband) {
      return 0.0;
    }

    // Normalize input to account for deadband
    double normalizedInput = (Math.abs(input) - deadband) / (1.0 - deadband);
    if (normalizedInput > 1.0) {
      normalizedInput = 1.0; // Clamp to ensure we don't exceed 1.0
    }

    // Apply the new S-curve formula: f(x) = 4x³-3x⁴
    // This gives a different acceleration profile than the standard smoothstep
    double x = normalizedInput;
    double processed = 4 * Math.pow(x, 3) - 3 * Math.pow(x, 4);

    // Return processed input with original sign
    return Math.copySign(processed, input);
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

  private void registerAutoCommands() {
    NamedCommands.registerCommand("L4", elevator.L4());
    NamedCommands.registerCommand("iceeSpitOut", icee.spitOut());
    NamedCommands.registerCommand("iceeIntake", icee.Intake());
    NamedCommands.registerCommand("iceeStop", icee.stop());
    NamedCommands.registerCommand("relocalize", drive.reLocalize());
    NamedCommands.registerCommand("L1", elevator.L1());
  }
}
