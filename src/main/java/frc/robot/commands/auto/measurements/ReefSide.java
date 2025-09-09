package frc.robot.commands.auto.measurements;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public enum ReefSide {
  ONE(18, 7),
  SIX(19, 6),
  FIVE(20, 11),
  FOUR(21, 10),
  THREE(22, 9),
  TWO(17, 8);

  public final Pose2d redTagPose;
  public final Pose2d blueTagPose;

  public Pose2d getCurrent() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? blueTagPose
        : redTagPose;
  }

  public ReefSide mirror() {
    switch (this) {
      case ONE:
        return ONE;
      case TWO:
        return SIX;
      case THREE:
        return FIVE;
      case FOUR:
        return FOUR;
      case FIVE:
        return THREE;
      default:
        return TWO; // SIX case
    }
  }

  private ReefSide(int blue, int red) {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    Optional<Pose2d> redPose = layout.getTagPose(red).map(Pose3d::toPose2d);
    Optional<Pose2d> bluePose = layout.getTagPose(blue).map(Pose3d::toPose2d);

    redTagPose =
        redPose.orElseThrow(() -> new IllegalArgumentException("Invalid red tag ID: " + red));
    blueTagPose =
        bluePose.orElseThrow(() -> new IllegalArgumentException("Invalid blue tag ID: " + blue));
  }
}
