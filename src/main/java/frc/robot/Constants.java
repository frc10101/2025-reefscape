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

  public static final class SparkMaxCan {
    public static int nDexterLeftID = 1;
    public static int nDexterRightID = 2;
    public static int ICEEID = 3;
  }

  public static final class IceeConstants {
    public static final double IceeInVelocity = 1;
    public static final double IceeOutVelocity = -1;
    public static final double Kp=0;
    public static final double Ki=0;
    public static final double Kd=0;
    public static final double ratio=1;
  } 

  public static final class NDexterConstants {
    public static final double rightFaster=1;
    public static final double rightSlower=.5;
    public static final double rightKp=0;
    public static final double rightKi=0;
    public static final double rightKd=0;
    public static final double rightGearRatio=5;

    public static final double leftFaster=1;
    public static final double leftSlower=.5;
    public static final double leftKp=0;
    public static final double leftKi=0;
    public static final double leftKd=0;
    public static final double leftGearRatio=9;








  }


}
