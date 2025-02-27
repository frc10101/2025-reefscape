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

  public static final class canIDs {
    /** Intake Can ID's */
    public static final int intakeMotor = 1;

    public static final int intakeMotorRotate = 2;
    public static final int intakeMotorFollower = 3;

    /** Elevator Can ID's */
    public static final int ElevatorMotor = 4;

    public static final int ElevatorMotorFollower = 5;
  }

  public static final class intakeConstants {
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kFF = 1.1;
    public static final double RotateKP = 1.3;
    public static final double RotateKI = 0.0;
    public static final double RotateKD = 0.7;
    public static final double RotateKFF = 1.1;
  }

  public static final class elevatorConstants {
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kFF = 1.1;
  }
}
