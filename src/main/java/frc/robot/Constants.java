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

  public static final class CANids {
    public static final int CANdleID = 25;
  }

  public static final class ArmConstants {
    // tuning variables
    public static final double kP = 3;
    public static final double kI = 0.0;

    public static final double kD = 1.2;
    public static final double kFF = 0;
    public static final double kFFwithCoral = 0;
    public static final double kFFTop = 0;
    public static final double kMaxAcceleration = 4500;
    public static final double kMaxVelocity = 4500;
    // arm measurements
    public static final double ARM_MASS_KG = 0.5; // kg
    public static final double ARM_LENGTH_METERS = 0.3; // meters
    public static final double GRAVITY_M_PER_SEC = 9.81; // m/s^2
    public static final double CENTER_OF_MASS = ARM_LENGTH_METERS / 2; // Center of mass
    public static final double GEAR_RATIO = 20.0; // Motor rotations per arm rotation
    public static final double kTorqueToVoltage = 0.1; // Tune: Nm to volts
    public static final double POWER_LIMIT = 0.5;

    // different positions (example values in radians)
    public static final double INTAKEPOSITION = 0.0; // 0 degrees
    public static final double HUMANPLAYERPOSITION = (Math.PI / 6) * -1; // -30 degrees
    public static final double L1POSITION = Math.PI / 6; // 30 degrees
    public static final double L2POSITION = 3; // 45 degrees
    public static final double L3POSITION = 3; // 60 degrees
    public static final double L4POSITION = 5 * Math.PI / 12; // 75 degrees
  }

  /** ALL SPARK MAX CAN ID'S = PDH PORT PLUS ONE */
  public static final class SparkMaxCanIDs {
    /** Intake Can ID's */
    public static final int intakeMotorSpinLeft = 10;

    public static final int intakeMotorSpinRight = 14;

    public static final int intakeMotorPivotLeft = 9;

    public static final int intakeMotorPivotRight = 17;

    /** Elevator Can ID's */
    public static final int ElevatorMotorLeft = 8;

    public static final int ElevatorMotorRight = 18;

    /** NDexter Can ID's */
    public static final int NDexterMotorLeft = 15;

    public static final int NDexterMotorRight = 16;

    /** Straw Can ID */
    public static final int StrawPivotMotor = 13;

    /** ICEE Can ID */
    public static final int IceeMotor = 12;
  }

  public static final class IntakeConstants {
    public static final double kMaxVelocity = 1.75;
    public static final double kMaxAcceleration = 0.75;
    public static final double RotateKMaxVelocity = 1.75;
    public static final double RotateKMaxAcceleration = 0.75;
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kFF = 1.1;
    public static final double RotateKP = 1.3;
    public static final double RotateKI = 0.0;
    public static final double RotateKD = 0.7;
    public static final double RotateKFF = 1.1;
    public static final double IntakeGearRatio = 1.5;
    public static final double IntakeUpPos = 5;
    public static final double IntakeDownPos = 0;
  }

  public static final class ElevatorConstants {
    public static final double kP = 0.17;
    public static final double kI = 0.0;
    public static final double kD = 0;
    public static final double kFF = 0.003;
    public static final double kMaxVelocity = 1.75;
    public static final double kMaxAcceleration = 0.75;
    public static final double ElevatorGearRatio = 20;
    public static final double NDexter = 0;
    public static final double goOut = -16;
    public static final double L1 = -1;
    public static final double L2 = -2;
    public static final double L3 = -9;
    public static final double L4 = -4;
    public static final double HumanPlayer = -5;
  }

  public static final class IceeConstants {
    public static final double IceeInVelocity = 1;
    public static final double IceeOutVelocity = -1;
    public static final double Kp = 0;
    public static final double Ki = 0;
    public static final double Kd = 0;
    public static final double ratio = 1;
    public static final double FF = 1.0 / 5767;
  }

  public static final class NDexterConstants {
    public static final double rightFaster = 1;
    public static final double rightSlower = .5;
    public static final double rightKp = 0;
    public static final double rightKi = 0;
    public static final double rightKd = 0;
    public static final double rightGearRatio = 5;
    public static final double leftFF = 1.0 / 5767;

    public static final double leftFaster = 1;
    public static final double leftSlower = .5;
    public static final double leftKp = 0;
    public static final double leftKi = 0;
    public static final double leftKd = 0;
    public static final double leftGearRatio = 9;
    public static final double rightFF = 1.0 / 5767;
  }
}
