// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSim;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class vision extends SubsystemBase {
  private VisionSystemSim visionSim = new VisionSystemSim("limelight");
  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private SimCameraProperties cameraprop = new SimCameraProperties();
  private PhotonCamera camera = new PhotonCamera("limelight");
  private PhotonCameraSim cameraSim;
  private DriveSim driveSim;

  /** Creates a new vision. */
  public vision(DriveSim driveSim) {
    visionSim.addAprilTags(aprilTagFieldLayout);
    cameraprop.setCalibration(1280, 800, Rotation2d.fromDegrees(92.4));
    cameraprop.setFPS(30);
    Translation3d robotToCameraTrl =
        new Translation3d(Inches.of(0.123), Inches.of(4.230), Inches.of(11.007));
    Rotation3d robotToCameraRot = new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180));
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
    cameraSim = new PhotonCameraSim(camera, cameraprop);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, robotToCamera);
    this.driveSim = driveSim;
  }

  @Override
  public void periodic() {
    visionSim.update(new Pose3d(driveSim.getPose()));
  }
}
