package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToPosition extends Command {
  private final PIDController xController, yController, rotController;
  private final Drive drivebase;
  private final Pose2d targetPose;

  public DriveToPosition(Drive drivebase, Pose2d targetpose) {
    this.xController = new PIDController(Constants.AutoDriveConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
    this.yController = new PIDController(Constants.AutoDriveConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);
    this.rotController = new PIDController(Constants.AutoDriveConstants.ROT_REEF_ALIGNMENT_P, 0, 0);

    this.xController.setTolerance(Constants.AutoDriveConstants.X_TOLERANCE_REEF_ALIGNMENT);
    this.yController.setTolerance(Constants.AutoDriveConstants.Y_TOLERANCE_REEF_ALIGNMENT);
    this.rotController.setTolerance(Constants.AutoDriveConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    this.drivebase = drivebase;
    this.targetPose = targetpose;

  }

  @Override
  public void initialize() {
    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotController.reset();
  }

  @Override
  public void execute() {
     // Get the current robot pose
     Pose2d currentPose = drivebase.getPose();

     // Calculate errors
     double xError = targetPose.getX() - currentPose.getX();
     double yError = targetPose.getY() - currentPose.getY();
     double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getDegrees();
 
     // Calculate PID outputs
     double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
     double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
     double rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());
 
     // Drive the robot using the calculated speeds
     drivebase.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, true);
 
     // Log data to AdvantageKit for replayability
     Logger.recordOutput("DriveToPosition/TargetPose", targetPose);
     Logger.recordOutput("DriveToPosition/CurrentPose", currentPose);
     Logger.recordOutput("DriveToPosition/XError", xError);
     Logger.recordOutput("DriveToPosition/YError", yError);
     Logger.recordOutput("DriveToPosition/RotationError", rotationError);
     Logger.recordOutput("DriveToPosition/XSpeed", xSpeed);
     Logger.recordOutput("DriveToPosition/YSpeed", ySpeed);
     Logger.recordOutput("DriveToPosition/RotSpeed", rotSpeed);
  }

  @Override
  public boolean isFinished() {
    // Check if the robot is within tolerances for position and rotation
    return atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    xController.reset();
    yController.reset();
    rotController.reset();
  }
  private boolean atSetpoint() {
    // Get the current robot pose
    Pose2d currentPose = drivebase.getPose();

    // Calculate the closest point of the robot to the target
    double robotClosestX = currentPose.getX() + (Constants.DriveConstants.BumperLength.magnitude() / 2.0) * Math.cos(currentPose.getRotation().getRadians());
    double robotClosestY = currentPose.getY() + (Constants.DriveConstants.BumperWidth.magnitude()/ 2.0) * Math.sin(currentPose.getRotation().getRadians());

    // Calculate errors for the closest point
    double xError = Math.abs(targetPose.getX() - robotClosestX);
    double yError = Math.abs(targetPose.getY() - robotClosestY);
    double rotationError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());

    // Check if the closest point is within tolerances
    boolean xAtSetpoint = xError <= Constants.AutoDriveConstants.X_TOLERANCE_REEF_ALIGNMENT;
    boolean yAtSetpoint = yError <= Constants.AutoDriveConstants.Y_TOLERANCE_REEF_ALIGNMENT;
    boolean rotationAtSetpoint = rotationError <= Constants.AutoDriveConstants.ROT_TOLERANCE_REEF_ALIGNMENT;

    // Log errors for debugging
    Logger.recordOutput("DriveToPosition/ClosestPointXError", xError);
    Logger.recordOutput("DriveToPosition/ClosestPointYError", yError);
    Logger.recordOutput("DriveToPosition/RotationError", rotationError);

    return xAtSetpoint && yAtSetpoint && rotationAtSetpoint;
}
}