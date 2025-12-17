package frc.robot.commands.auto;

import static frc.robot.subsystems.drive.DriveConstants.kAlignMaxSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;

/** A command that aligns the robot to a certain field-relative position */
public class AlignToPose extends Command {
  // This command works by using a simple PID loop to move x/y positions and rotate angle
  private PIDController xPidController, yPidController, thetaPidController;
  private double xP, xI, xD;
  private double yP, yI, yD;
  private double thetaP, thetaI, thetaD;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private Drive drive;
  private boolean isAuto;

  /**
   * A command that aligns the robot to a certain field-relative position
   *
   * @param drive The (swerve) drivetrain subsystem
   * @param targetPoseSupplier A function returning the desired position (field-relative, using blue
   *     as the origin)
   */
  public AlignToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier, boolean isAuto) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.isAuto = isAuto;

    addRequirements(drive);

    xP = yP = DriveConstants.kTranslationP;
    xI = yI = DriveConstants.kTranslationI;
    xD = yD = DriveConstants.kTranslationD;

    thetaP = DriveConstants.kTurnAngleP;
    thetaI = DriveConstants.kTurnAngleI;
    thetaD = DriveConstants.kTurnAngleD;

    xPidController = new PIDController(xP, xI, xD);
    yPidController = new PIDController(yP, yI, yD);
    thetaPidController = new PIDController(thetaP, thetaI, thetaD);

    thetaPidController.enableContinuousInput(0, 2 * Math.PI);
  }

  @Override
  public void initialize() {
    // Sets position once at start of command
    targetPose = targetPoseSupplier.get();

    // Resets state and integral term of PID controllers
    xPidController.reset();
    yPidController.reset();
    thetaPidController.reset();

    // Set the setpoints once at the start of the command
    // Prevents rapidly oscillating movement by going to only 1 setpoint at a time
    xPidController.setSetpoint(targetPoseSupplier.get().getX());
    yPidController.setSetpoint(targetPoseSupplier.get().getY());
    thetaPidController.setSetpoint(targetPoseSupplier.get().getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    // Field-relative PID calculations for how much to move in x and y directions
    double xOutput =
        xPidController.calculate(currentPose.getX()) * DriveConstants.kMaxSpeedMetersPerSecond;
    double yOutput =
        -1 * yPidController.calculate(currentPose.getY()) * DriveConstants.kMaxSpeedMetersPerSecond;

    // Normalize x and y velocity vectors
    // if they want the robot to move faster than our constraint says it can
    double magnitude = Math.hypot(xOutput, yOutput);
    if (magnitude > kAlignMaxSpeed) {
      xOutput = xOutput / magnitude * kAlignMaxSpeed;
      yOutput = yOutput / magnitude * kAlignMaxSpeed;
    }

    // Apply small deadband to prevent swerve tweaking
    if (magnitude < 0.01) {
      xOutput = 0;
      yOutput = 0;
    }

    // PID calculation for how much to turn
    double thetaOutput =
        MathUtil.clamp(
            thetaPidController.calculate(currentPose.getRotation().getRadians())
                * drive.getMaxAngularSpeedRadPerSec(),
            -DriveConstants.kAlignMaxAngularSpeed,
            DriveConstants.kAlignMaxAngularSpeed);

    // Convert field-relative speeds to robot-relative speeds
    ChassisSpeeds driveSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, drive.getRotation());

    drive.runVelocity(driveSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    if (isAuto) {
      // Finish if distance between current and desired position is close enough
      return drive.getPose().getTranslation().minus(targetPose.getTranslation()).getNorm()
              <= DriveConstants.kAlignPositionTolerance
          && drive.getRotation().getRadians() - targetPose.getRotation().getRadians()
              <= DriveConstants.kAlignRotationTolerance;
    }
    return false;
  }
}
