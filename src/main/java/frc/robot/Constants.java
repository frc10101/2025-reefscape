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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class CANids {
    public static final int CANdleID = 25;
  }

  public static final class digitalIDs {
    public static final int daisyBeamBreak = 0;
  }

  /** ALL SPARK MAX CAN ID'S = PDH PORT PLUS ONE */
  public static final class SparkMaxCanIDs {
    /** Daisy Can ID's */
    public static final int DaisyMotor = 17;

    /** Elevator Can ID's */
    public static final int ElevatorMotorLeft = 8;

    public static final int ElevatorMotorRight = 2;
  }

  public static final class ElevatorConstants {
    public static final double kP = 0.17;
    public static final double kI = 0.0;
    public static final double kD = 0;
    public static final double kFF = 0.003;
    public static final double kMaxVelocity = 1.75;
    public static final double kMaxAcceleration = 0.75;
    public static final double ElevatorGearRatio = 25;
    public static final double NDexter = 0;
    public static final double goOut = -16;
    public static final double L1 = -4;
    public static final double L2 = -11.25;
    public static final double L3 = -20.25;
    public static final double HumanPlayer = 0;
    public static final double kCarriageMass = 5.31; //kg
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.88);
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = Units.feetToMeters(4.5);
    public static final double kPixelsPerMeter = 20.0; // pixels per meter conversion factor
  }

  public static final class DaisyConstants {
    public static final double DaisyIn = 1; // arbitrary number
    public static final double DaisyOut = -1; // also and arbitrary number
  }

  public static final class LimelightConstants {
    public static final String limelightName = "Johnny";
  }
}
