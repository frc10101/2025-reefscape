// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SparkMaxCan;

public class NDexter extends SubsystemBase {
  /** Creates a new nDexter. */
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private boolean canSpin=true;

  public NDexter() {
    this.leftMotor = new SparkMax(SparkMaxCan.nDexterLeftID, MotorType.kBrushless);
    this.rightMotor = new SparkMax(SparkMaxCan.nDexterRightID, MotorType.kBrushless);
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();


    leftMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1/Constants.NDexterConstants.leftGearRatio);

    rightMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1/Constants.NDexterConstants.rightGearRatio);



    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.NDexterConstants.leftKp)
        .i(Constants.NDexterConstants.leftKi)
        .d(Constants.NDexterConstants.leftKd)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rightMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.NDexterConstants.rightKp)
        .i(Constants.NDexterConstants.rightKi)
        .d(Constants.NDexterConstants.rightKd)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command runSame() {
    if(canSpin){
      return runOnce(
          () -> {
            this.rightMotor.getClosedLoopController().setReference(
                Constants.NDexterConstants.rightFaster,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0);
            this.leftMotor.getClosedLoopController().setReference(
                Constants.NDexterConstants.leftFaster,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0);
          });
    }
    else{
      return this.stop();
    }
  }

  public Command rightFaster() {
    if(canSpin){
      return runOnce(
          () -> {
            this.rightMotor.getClosedLoopController().setReference(
              Constants.NDexterConstants.rightFaster,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0);
            this.leftMotor.getClosedLoopController().setReference(
              Constants.NDexterConstants.leftSlower,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0);
          });
    }
    else{
      return this.stop();
    }
  } 


  public Command leftFaster() {
    if(canSpin){
      return runOnce(
          () -> {
            this.rightMotor.getClosedLoopController().setReference(
                Constants.NDexterConstants.rightSlower,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0);
            this.leftMotor.getClosedLoopController().setReference(
              Constants.NDexterConstants.leftFaster,
                ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0);
          });
    }
    else{
      return this.stop();
    }
  }

  public Command stop() {

    return runOnce(
        () -> {
          this.rightMotor.getClosedLoopController().setReference(
              0,
              ControlType.kMAXMotionVelocityControl,
              ClosedLoopSlot.kSlot0);
          this.leftMotor.getClosedLoopController().setReference(
              0,
              ControlType.kMAXMotionVelocityControl,
              ClosedLoopSlot.kSlot0);
        });
  }

  public Command Out() {
    return runOnce(
        () -> {
          this.rightMotor.getClosedLoopController().setReference(
              -Constants.NDexterConstants.rightFaster,
              ControlType.kMAXMotionVelocityControl,
              ClosedLoopSlot.kSlot0);
          this.leftMotor.getClosedLoopController().setReference(
            -Constants.NDexterConstants.leftFaster,
              ControlType.kMAXMotionVelocityControl,
              ClosedLoopSlot.kSlot0);
        });
  }

  public Command canSpin(boolean spin){
    return runOnce(()->{
     canSpin=spin;
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
