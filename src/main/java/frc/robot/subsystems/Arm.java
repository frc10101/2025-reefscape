package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  //private double thetaRef; // Desired arm angle (radians)
  //private double theta; // Current arm angle (radians)

  CommandJoystick joystick = ;

  private final SparkMax armMotor;

  public Arm() {
    armMotor = new SparkMax(Constants.SparkMaxIDs.ArmMotor, MotorType.kBrushless);
    armMotor.getEncoder().setPosition(0);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.closedLoop.pidf(
        Constants.ArmConstants.kP,
        Constants.ArmConstants.kI,
        Constants.ArmConstants.kD,
        Constants.ArmConstants.kFF);

    armConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(Constants.ArmConstants.kMaxAcceleration)
        .maxVelocity(Constants.ArmConstants.kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    armConfig.encoder.positionConversionFactor(2.0 * Math.PI / ArmConstants.GEAR_RATIO);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public void IntakePositoin(){



    armMotor
    .getClosedLoopController()
    .setReference(
          * 0.1, // tune the 0.1 as a scaling factor
        ControlType.kMAXMotionPositionControl);
  }

  public void stop() {
    armMotor.set(0);
  }

  public void initDefaultCommand(){

  }


}
