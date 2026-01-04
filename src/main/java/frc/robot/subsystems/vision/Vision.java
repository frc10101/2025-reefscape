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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public double getFiducialID(int cameraIndex) {
      // Check if there are any pose observations available
      if (inputs[cameraIndex].poseObservations.length == 0) {
          return -1; // Return -1 if no tags are detected
      }
  
      // Find the closest tag based on averageTagDistance
      VisionIO.PoseObservation closestObservation = inputs[cameraIndex].poseObservations[0];
      for (VisionIO.PoseObservation observation : inputs[cameraIndex].poseObservations) {
          if (observation.averageTagDistance() < closestObservation.averageTagDistance()) {
              closestObservation = observation;
          }
      }
  
      // Return the ID of the closest tag
      if (closestObservation.tagCount() > 0) {
          return inputs[cameraIndex].tagIds[0]; // Assuming tagIds correspond to poseObservations
      }
  
      return -1; // Return -1 if no valid tag is found
  }
  
    public boolean getTV(int cameraIndex) {
      return inputs[cameraIndex].tagIds.length > 0;
    }
    /**
 * Returns the 3D transform of the robot in the coordinate system of the primary in-view AprilTag.
 * The result is an array of six elements: [tx, ty, tz, pitch, yaw, roll].
 *
 * @param cameraIndex The index of the camera to use.
 * @return An array representing the robot's pose in the target's coordinate system:
 *         [tx, ty, tz, pitch, yaw, roll] (meters, degrees).
 */
public double[] getBotPose_TargetSpace(int cameraIndex) {
  // Check if there are any tag IDs available
  if (inputs[cameraIndex].tagIds.length == 0) {
      return new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Return zero array if no tag is detected
  }

  // Get the primary tag ID and its pose
  int primaryTagId = inputs[cameraIndex].tagIds[0];
  var tagPoseOptional = aprilTagLayout.getTagPose(primaryTagId);

  if (tagPoseOptional.isEmpty()) {
      return new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Return zero array if tag pose is unavailable
  }

  Pose3d tagPose = tagPoseOptional.get();

  // Get the robot's pose relative to the camera
  if (inputs[cameraIndex].poseObservations.length == 0) {
      return new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Return zero array if no pose observations
  }

  Pose3d robotPoseRelativeToCamera = inputs[cameraIndex].poseObservations[0].pose();

  // Transform the robot's pose into the tag's coordinate system
  Pose3d robotPoseInTagSpace = tagPose.relativeTo(robotPoseRelativeToCamera);

  // Extract translation and rotation components
  double tx = robotPoseInTagSpace.getX(); // Translation X (meters)
  double ty = robotPoseInTagSpace.getY(); // Translation Y (meters)
  double tz = robotPoseInTagSpace.getZ(); // Translation Z (meters)

  double pitch = Math.toDegrees(robotPoseInTagSpace.getRotation().getX()); // Pitch (degrees)
  double yaw = Math.toDegrees(robotPoseInTagSpace.getRotation().getY());   // Yaw (degrees)
  double roll = Math.toDegrees(robotPoseInTagSpace.getRotation().getZ());  // Roll (degrees)

  // Return the result as an array
  return new double[] {tx, ty, tz, pitch, yaw, roll};
}

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            String path = "Vision/Camera" + Integer.toString(i);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) /
observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new
Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new
Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1>
visionMeasurementStdDevs);
    }
}
