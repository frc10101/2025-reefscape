package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final double ARM_MASS = 0.5; // kg
  private final double ARM_LENGTH = 0.3; // meters
  private final double GRAVITY = 9.81; // m/s^2
  private final double L_C = ARM_LENGTH / 2; // Center of mass
  private final double GEAR_RATIO = 10.0; // Motor rotations per arm rotation
  private final double kTorqueToVoltage = 0.1; // Tune: Nm to volts

  private double thetaRef; // Desired arm angle (radians)
  private double theta; // Current arm angle (radians)

  private final SparkMax armMotor;

  public Arm() {
    armMotor = new SparkMax(Constants.canIDs.ArmMotor, MotorType.kBrushless);
    armMotor.getEncoder().setPosition(0);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.closedLoop.pidf(
        Constants.armConstants.kP,
        Constants.armConstants.kI,
        Constants.armConstants.kD,
        Constants.armConstants.kFF);

    armConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(Constants.armConstants.kMaxAcceleration)
        .maxVelocity(Constants.armConstants.kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    armConfig.encoder.positionConversionFactor(2.0 * Math.PI / GEAR_RATIO);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    thetaRef = 0.0;
  }

  public double getAngle() {
    return theta;
  }

  public void stop() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    theta = armMotor.getEncoder().getPosition();

    // calculate gravity feed forward in volts
    double gravityFF = (ARM_MASS * GRAVITY * L_C * Math.cos(thetaRef)) * kTorqueToVoltage;
    armMotor
        .getClosedLoopController()
        .setReference(
            thetaRef + gravityFF * 0.1, // tune the 0.1 as a scaling factor
            ControlType.kMAXMotionPositionControl);
  }
}
