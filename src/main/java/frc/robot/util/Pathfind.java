// Copyright (c) 2025 FRC Team 1466
// https://github.com/FRC1466

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.text.ParseException;

public class Pathfind {
  PathConstraints constraints;
  // sendable chooser for pathfinding testing
  public static SendableChooser<Pose2d> testPoseChooser = new SendableChooser<>();

  public Pathfind() throws IOException, ParseException {
    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only be used for the path.
    constraints =
        new PathConstraints(5, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  public Command pathToPose(Pose2d desirPose) {
    return AutoBuilder.pathfindToPose(desirPose, constraints, 0);
  }
}
