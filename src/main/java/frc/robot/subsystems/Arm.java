package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final SparkMax armMotor;

  public Arm() {
    armMotor = new SparkMax(Constants.SparkMaxCanIDs.StrawPivotMotor, MotorType.kBrushless);
    armMotor.getEncoder().setPosition(0);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.closedLoop.pidf(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kFF);

    armConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(ArmConstants.kMaxAcceleration)
        .maxVelocity(ArmConstants.kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    armConfig.encoder.positionConversionFactor(2.0 * Math.PI / ArmConstants.GEAR_RATIO);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public void stop() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // No continuous updates needed
  }

  public Command setArmPosition(double position) {
    return new Command() {
      private double targetPosition;

      @Override
      public void initialize() {
        targetPosition = position;
        armMotor
            .getClosedLoopController()
            .setReference(position, ControlType.kMAXMotionPositionControl);
      }

      @Override
      public boolean isFinished() {
        double currentPosition = armMotor.getEncoder().getPosition();
        return Math.abs(currentPosition - targetPosition) < 0.05;
      }

      @Override
      public void end(boolean interrupted) {
        if (interrupted) {
          stop();
        }
      }
    }.withName("SetArmPosition");
  }
}
