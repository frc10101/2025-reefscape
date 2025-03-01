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

  public static final class SparkMaxIDs {
    public static final int ArmMotor = 6;
  }

  public static final class ArmConstants {
    // tuning variables
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kFF = 0.0;
    public static final double kMaxAcceleration = 2.0;
    public static final double kMaxVelocity = 1.0;

    // arm measurements
    public static final double ARM_MASS_KG = 0.5; // kg
    public static final double ARM_LENGTH_METERS = 0.3; // meters
    public static final double GRAVITY_M_PER_SEC = 9.81; // m/s^2
    public static final double CENTER_OF_MASS = ARM_LENGTH_METERS / 2; // Center of mass
    public static final double GEAR_RATIO = 1.0; // Motor rotations per arm rotation
    public static final double kTorqueToVoltage = 0.1; // Tune: Nm to volts

    // different positions (example values in radians)
    public static final double INTAKEPOSITION = 0.0; // 0 degrees
    public static final double HUMANPLAYERPOSITION = Math.PI / 6; // 30 degrees
    public static final double L1POSITION = Math.PI / 3; // 60 degrees
    public static final double L2POSITION = Math.PI / 2; // 90 degrees
    public static final double L3POSITION = 2 * Math.PI / 3; // 120 degrees
    public static final double L4POSITION = 5 * Math.PI / 6;
  }
}
