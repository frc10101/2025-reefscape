// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SparkMaxCan;

public class nDexter extends SubsystemBase {
  /** Creates a new nDexter. */
  public static final double ki = 0;

  public static final double kp = 0.1;
  public static final double kd = 0.001;

  private SparkMax leftSide;
  private SparkMax rightSide;

  private double targetVelocityLeft;
  private double targetVelocityRight;

  private PIDController leftController;
  private PIDController rightController;

  private SparkMaxConfig rightMotorConfig;
  private SparkMaxConfig leftMotorConfig;

  private SparkClosedLoopController rightClosedLoopController;
  private SparkClosedLoopController leftClosedLoopController;

  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;

  public nDexter() {
    this.leftSide = new SparkMax(SparkMaxCan.nDexterLeftID, MotorType.kBrushless);
    this.rightSide = new SparkMax(SparkMaxCan.nDexterRightID, MotorType.kBrushless);
    this.targetVelocityLeft = 0;
    this.targetVelocityRight = 0;
    this.leftController = new PIDController(kp, ki, kd);
    this.rightController = new PIDController(kp, ki, kd);

    this.rightClosedLoopController = this.rightSide.getClosedLoopController();
    this.leftClosedLoopController = this.leftSide.getClosedLoopController();

    this.rightEncoder = rightSide.getEncoder();
    this.leftEncoder = leftSide.getEncoder();

    rightMotorConfig = new SparkMaxConfig();
    leftMotorConfig = new SparkMaxConfig();

    leftMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    rightMotorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    leftSide.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rightMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    rightSide.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setTargetVelocity(double leftTarget, double rightTarget) {
    this.targetVelocityLeft = leftTarget;
    this.targetVelocityRight = rightTarget;
  }

  public Command runOnPID() {

    return runOnce(
        () -> {
          leftController.setSetpoint(this.targetVelocityLeft);
          rightController.setSetpoint(this.targetVelocityRight);

          // leftSide.setVoltage(leftController.calculate(leftSide.get()));
          // rightSide.setVoltage(rightController.calculate(rightSide.get()));
          rightClosedLoopController.setReference(
              this.targetVelocityRight,
              ControlType.kMAXMotionVelocityControl,
              ClosedLoopSlot.kSlot0);
          leftClosedLoopController.setReference(
              this.targetVelocityLeft,
              ControlType.kMAXMotionVelocityControl,
              ClosedLoopSlot.kSlot0);
        });
  }

  // public boolean overDraw(){
  //   if(leftSide.getBusVoltage() > MARGIN || rightSide.getBusVoltage() > MARGIN){
  //     return true;
  //   }
  //   return false;
  // }

  public void assighnPower(double left, double right) {
    leftSide.setVoltage(left);
    rightSide.setVoltage(right);
  }

  public Command power(double left, double right) {
    return runOnce(() -> this.assighnPower(left, right));
  }

  public void setTarget(double leftTarget, double rightTarget) {
    this.targetVelocityLeft = leftTarget;
    this.targetVelocityRight = rightTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
