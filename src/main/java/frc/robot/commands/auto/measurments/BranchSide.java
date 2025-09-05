package frc.robot.commands.auto.measurments;

import edu.wpi.first.math.geometry.Translation2d;

public enum BranchSide {
  LEFT(new Translation2d(-0.153209, 0.5406845 + 0.02)),
  RIGHT(new Translation2d(0.218062 - 0.0508, 0.5408565 + 0.02)),
  MIDDLE(new Translation2d(0.064853, 0.5408565 + 0.02));

  public Translation2d tagOffset;

  private BranchSide(Translation2d offsets) {
    tagOffset = offsets;
  }

  public BranchSide mirror() {
    switch (this) {
      case LEFT:
        return RIGHT;
      case MIDDLE:
        return MIDDLE;
      default:
        return LEFT;
    }
  }
}
