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
import frc.robot.Constants.SparkMaxCanIDs;

import org.littletonrobotics.junction.Logger;

public class NDexter extends SubsystemBase {
  /** Creates a new nDexter. */
  private final SparkMax leftMotor;

  private final SparkMax rightMotor;
  private boolean canSpin = true;

  public NDexter() {
    this.leftMotor = new SparkMax(SparkMaxCanIDs.NDexterMotorLeft, MotorType.kBrushless);
    this.rightMotor = new SparkMax(SparkMaxCanIDs.NDexterMotorRight, MotorType.kBrushless);
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    leftMotorConfig.encoder.velocityConversionFactor(1 / Constants.NDexterConstants.leftGearRatio);

    leftMotorConfig.smartCurrentLimit(60);
    rightMotorConfig.smartCurrentLimit(60);

    rightMotorConfig.encoder.velocityConversionFactor(
        1 / Constants.NDexterConstants.rightGearRatio);

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
        .velocityFF(Constants.NDexterConstants.leftFF);

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
        .velocityFF(Constants.NDexterConstants.rightFF)
        .outputRange(-1, 1);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command runSame() {
    if (!canSpin) {
      return this.stop();
    }

    return runOnce(
        () -> {
          this.rightMotor
              .getClosedLoopController()
              .setReference(
                  Constants.NDexterConstants.rightFaster,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
          this.leftMotor
              .getClosedLoopController()
              .setReference(
                  Constants.NDexterConstants.leftFaster,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
        });
  }

  public Command rightFaster() {
    if (!canSpin) {
      return this.stop();
    }
    return runOnce(
        () -> {
          this.rightMotor
              .getClosedLoopController()
              .setReference(
                  Constants.NDexterConstants.rightFaster,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
          this.leftMotor
              .getClosedLoopController()
              .setReference(
                  Constants.NDexterConstants.leftSlower,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
        });
  }

  public Command leftFaster() {
    if (!canSpin) {
      return this.stop();
    }
    return runOnce(
        () -> {
          this.rightMotor
              .getClosedLoopController()
              .setReference(
                  Constants.NDexterConstants.rightSlower,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
          this.leftMotor
              .getClosedLoopController()
              .setReference(
                  Constants.NDexterConstants.leftFaster,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
        });
  }

  public Command stop() {

    return runOnce(
        () -> {
          this.rightMotor
              .getClosedLoopController()
              .setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          this.leftMotor
              .getClosedLoopController()
              .setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        });
  }

  public Command Out() {
    return runOnce(
        () -> {
          this.rightMotor
              .getClosedLoopController()
              .setReference(
                  -Constants.NDexterConstants.rightFaster,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
          this.leftMotor
              .getClosedLoopController()
              .setReference(
                  -Constants.NDexterConstants.leftFaster,
                  ControlType.kVelocity,
                  ClosedLoopSlot.kSlot0);
        });
  }

  public Command canSpin(boolean spin) {
    return runOnce(
        () -> {
          canSpin = spin;
        });
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Left NDexter Motor", leftMotor.get());
    Logger.recordOutput("Right NDexter Motor", leftMotor.get());
  }
}
