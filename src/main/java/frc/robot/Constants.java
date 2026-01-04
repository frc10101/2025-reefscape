package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Set;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Defines constants for the robot, including runtime modes, CAN IDs, and subsystem-specific values.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    REAL, // Running on a real robot
    SIM, // Running a physics simulator
    REPLAY // Replaying from a log file
  }

  public static final class CANids {
    public static final int CANdleID = 25;
  }

  public static final class DigitalIDs {
    public static final int daisyBeamBreak = 0;
  }

  public static final class SparkMaxCanIDs {
    public static final int DaisyMotor = 16;
    public static final int ElevatorMotorLeft = 8;
    public static final int ElevatorMotorRight = 18;
  }

  public static final class ElevatorConstants {
    public static final double kP = 0.17;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.003;
    public static final double kMaxVelocity = 1.75;
    public static final double kMaxAcceleration = 0.75;
    public static final double ElevatorGearRatio = 25.0;
    public static final double L1 = -4.0;
    public static final double L2 = -11.25;
    public static final double L3 = Units.inchesToMeters(43);
    public static final double HumanPlayer = 0.0;
    public static final double kCarriageMass = 5.31; // kg
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.88);
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = Units.feetToMeters(7.0);
    public static final double kS = 0.2; // Static friction feedforward
    public static final double kG = 0.8; // Gravity feedforward
    public static final double kV = 0.1; // Velocity feedforward
    public static final double kCurrentLimit = 40.0; // Amps
    public static final double kSimClosedLoopP = 6.189;
    public static final double kSimClosedLoopI = 0.0;
    public static final double kSimClosedLoopD = 0.0;
    public static final double kSimFeedforwardS = 0.0;
    public static final double kSimFeedforwardG = 0.04;
    public static final double kSimFeedforwardV = 38.19;
    public static final double kSimFeedforwardA = 0.0;
    public static final double kClosedLoopRampRate = 0.25; // seconds
    public static final double kRaiseSpeed = 0.5;
    public static final double kLowerSpeed = -0.5;
    public static final double kStopSpeed = 0.0;
    public static final double kEjectHeightAboveDaisy = Units.inchesToMeters(15);
    public static final double kEjectSpeed = 2.0; // meters per second
    public static final double kEjectAngle = -35.0; // degrees
    public static final double kMaxElevatorPoseHeightMeters = Units.inchesToMeters(19);
    public static final double kMaxDaisyPoseHeightMeters = Units.inchesToMeters(43);
  }

  public static final class DaisyConstants {
    public static final double DaisyIn = 1.0;
    public static final double DaisyOut = -1.0;
    public static final double DaisyInRPM = -300.0; // RPM
    public static final double DaisyOutRPM = 650; // RPM
    public static final double kDiameterMeters = 0.0762; // Diameter of the flywheel in meters
    public static final double kMassPounds = 1.2; // Mass of the flywheel in pounds
    public static final double kPID_P = 0.0025; // PID proportional constant
    public static final double kPID_I = 0.0; // PID integral constant
    public static final double kPID_D = 0.0001; // PID derivative constant
    public static final double kStatorCurrentLimitAmps = 40.0; // Current limit in amps
    public static final double kGearRatio = 1.0; // Gear ratio for the Daisy mechanism
    public static final double kFeedforwardS = 0.05; // Static gain for feedforward
    public static final double kFeedforwardV = 0.51; // Velocity gain for feedforward
    public static final double kFeedforwardA = 0.034; // Acceleration gain for feedforward
  }

  public static final class AutoDriveConstants {
    public static final double X_REEF_ALIGNMENT_P = 5;
    public static final double Y_REEF_ALIGNMENT_P = 5;
    public static final double ROT_REEF_ALIGNMENT_P = 0.1;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34; // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;
    public static final Set<Integer> REEF_TAG_IDS = Set.of(6, 7, 8, 9,10,17,18,19,20,21,22);
  }

  public static final class LimelightConstants {
    public static final String limelightName = "Johnny";
  }

  public static final class DriveConstants {
    public static final double MaxSpeed = 3.0; // Maximum speed in meters per second
    public static final double MaxAngularRate =
        Math.PI * 0.75; // Maximum angular rate in radians per second
    public static final double MaxAngularAcceleration = Math.PI; // Maximum angular acceleration in radians per second squared
    public static final Distance TrackLength = Inches.of(23.0); //x Track length in inches
    public static final Distance TrackWidth = Inches.of(23.0); //y Track width in inches
    public static final Distance BumperLength = Inches.of(34.0); //x Track length in inches
    public static final Distance BumperWidth = Inches.of(34.0); //y Track width in inches
  }
}
