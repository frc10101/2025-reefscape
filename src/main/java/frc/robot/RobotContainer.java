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

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.YamsDaisy;
import frc.robot.subsystems.YamsElevator;
import frc.robot.subsystems.drive.DriveSim;
import frc.robot.subsystems.visionSim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // MapleSim testing
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive2 =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);
  // Subsystems
  // public final Drive drive = TunerConstants.createDrivetrain();
  public final DriveSim drive = TunerConstants.createDrivetrain();
  public final YamsElevator elevator = new YamsElevator();
  public final YamsDaisy daisy = new YamsDaisy();
  public final visionSim vision = new visionSim(drive);
  // private final Elevator elevator = new Elevator();
  // private final Daisy daisy = new Daisy();

  // Controllers
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  // private final CommandPS4Controller controller2 = new CommandPS4Controller(1);

  // Crit Hit way
  // private final LoggedDashboardChooser<Command> autoChooser;

  // Maplesim way
  private final SendableChooser<Command> autoChooser;

  // Field
  private final Field2d m_field = new Field2d();

  public RobotContainer() {
    SmartDashboard.putData("Field", m_field);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    // SmartDashboard.putData("Field", m_field);

    configureBindings();

    drive.resetPose(new Pose2d(7.16, 5, new Rotation2d(Math.PI)));
    SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2,2)));
    daisy.setDefaultCommand(daisy.setVelocity(RPM.of(0)));


    /*
        drive.resetPose(new Pose2d(7.16, 5, new Rotation2d(Math.PI)));

        // adds all autos (ALL OF THEM)
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("Field", m_field);

        // PathPlanner Commands
        NamedCommands.registerCommand("L2", elevator.L2());
        NamedCommands.registerCommand("L3", elevator.L3());
        NamedCommands.registerCommand("HumanPlayer", elevator.HumanPlayer());

        NamedCommands.registerCommand(
            "outputSpin", daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
        NamedCommands.registerCommand("stopSpin", daisy.outputSpin(0));

        // Configure button bindings
        configureButtonBindings();
    */
  }

  /*private void configureButtonBindings() {
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
      // Trigger button10 = new Trigger(controller.button(10)); //ResetGyro
      // button1.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyIn));
      // button2.whileTrue(daisy.outputSpin(Constants.DaisyConstants.DaisyOut));
      // button3.whileTrue(elevator.raise());
      button4.whileTrue(elevator.lower());
      button6.whileTrue(elevator.L3());
      button7.whileTrue(elevator.HumanPlayer());
      button9.whileTrue(elevator.L2());
      // button10.whileTrue(elevator.L1());
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
          .button(1) // should be a
          .whileTrue(
              DriveCommands.joystickDriveAtAngle(
                  drive,
                  () -> controller.getLeftY(),
                  () -> controller.getLeftX(),
                  () -> new Rotation2d()));

      // Switch to X pattern when X button is pressed
      controller.button(4).onTrue(Commands.runOnce(drive::stopWithX, drive));

      // should be b
      controller.button(5).onTrue(drive.runOnce(() -> drive.seedFieldCentric()));
    }
  */

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drive.applyRequest(
            () ->
                drive2
                    .withVelocityX(
                        -controller.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -controller.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    controller.button(2).whileTrue(drive.applyRequest(() -> brake));
    controller
        .button(3)
        .whileTrue(
            drive.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

    controller
        .pov(0)
        .whileTrue(drive.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    controller
        .pov(180)
        .whileTrue(drive.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // reset the field-centric heading on left bumper press
    controller.button(5).onTrue(drive.runOnce(() -> drive.seedFieldCentric()));
    controller.triangle().onTrue(elevator.L3());
    controller.cross().onTrue(elevator.HumanPlayer());
    controller.R1().onTrue(elevator.ejectCoral(drive, daisy));
    controller.square().onTrue(daisy.setVelocity(RPM.of(Constants.DaisyConstants.DaisyOutRPM)));
    controller.circle().onTrue(daisy.setVelocity(RPM.of(Constants.DaisyConstants.DaisyInRPM)));
    

    drive.registerTelemetry(logger::telemeterize);
    Logger.recordOutput("zeroedPose", new Pose3d());
  }

  // public void zeroGyro() {
  //  drive.runOnce(() -> drive.seedFieldCentric());
  // }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
