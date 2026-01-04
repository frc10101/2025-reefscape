// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.visionSim;
import frc.robot.subsystems.drive.DriveSim;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSim drivebase;
  private visionSim vision;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, DriveSim drivebase, visionSim vision) {
    xController = new PIDController(Constants.AutoDriveConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.AutoDriveConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.AutoDriveConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.vision = vision;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.AutoDriveConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.AutoDriveConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.AutoDriveConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.AutoDriveConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.AutoDriveConstants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.AutoDriveConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.AutoDriveConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = vision.getFiducialID();
  }

  @Override
  public void execute() {
    if (vision.getTV() && vision.getFiducialID() == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = vision.getBotPose_TargetSpace();
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.AutoDriveConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.AutoDriveConstants.POSE_VALIDATION_TIME);
  }
}