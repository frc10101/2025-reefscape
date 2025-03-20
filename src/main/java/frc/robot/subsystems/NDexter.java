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
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private boolean canSpin = true;

  public NDexter() {
    leftMotor = configureMotor(SparkMaxCanIDs.NDexterMotorLeft, false, Constants.NDexterConstants.leftGearRatio, Constants.NDexterConstants.leftKp, Constants.NDexterConstants.leftKi, Constants.NDexterConstants.leftKd, Constants.NDexterConstants.leftFF);
    rightMotor = configureMotor(SparkMaxCanIDs.NDexterMotorRight, true, Constants.NDexterConstants.rightGearRatio, Constants.NDexterConstants.rightKp, Constants.NDexterConstants.rightKi, Constants.NDexterConstants.rightKd, Constants.NDexterConstants.rightFF);
  }

  private SparkMax configureMotor(int canID, boolean inverted, double gearRatio, double kp, double ki, double kd, double ff) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder.velocityConversionFactor(1 / gearRatio);
    config.smartCurrentLimit(60);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kp)
        .i(ki)
        .d(kd)
        .velocityFF(ff)
        .outputRange(-1, 1);
    config.inverted(inverted);

    SparkMax motor = new SparkMax(canID, MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    return motor;
  }

  private Command setMotorVelocities(double leftVelocity, double rightVelocity) {
    if (!canSpin) {
      return stop();
    }
    return runOnce(() -> {
      leftMotor.getClosedLoopController().setReference(leftVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
      rightMotor.getClosedLoopController().setReference(rightVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    });
  }

  public Command runSame() {
    return setMotorVelocities(Constants.NDexterConstants.leftFaster, Constants.NDexterConstants.rightFaster);
  }

  public Command rightFaster() {
    return setMotorVelocities(Constants.NDexterConstants.leftSlower, Constants.NDexterConstants.rightFaster);
  }

  public Command leftFaster() {
    return setMotorVelocities(Constants.NDexterConstants.leftFaster, Constants.NDexterConstants.rightSlower);
  }

  public Command stop() {
    return runOnce(() -> {
      leftMotor.set(0);
      rightMotor.set(0);
    });
  }

  public Command Out() {
    return setMotorVelocities(-Constants.NDexterConstants.leftFaster, -Constants.NDexterConstants.rightFaster);
  }

  public Command canSpin(boolean spin) {
    return runOnce(() -> canSpin = spin);
  }

  private Command runMotors(double leftSpeed, double rightSpeed) {
    return runEnd(() -> {
      leftMotor.set(leftSpeed);
      rightMotor.set(rightSpeed);
    }, this::stopMotors);
  }

  public Command NDexterSpinSameForward() {
    return runMotors(0.3, 0.3);
  }

  public Command NDexterSpinLeftFasterForward() {
    return runMotors(0.3, 0.2);
  }

  public Command NDexterSpinRightFasterForward() {
    return runMotors(0.2, 0.3);
  }

  public Command NDexterSpinSameReverse() {
    return runMotors(-0.3, -0.3);
  }

  public Command NDexterSpinLeftFasterReverse() {
    return runMotors(-0.3, -0.2);
  }

  public Command NDexterSpinRightFasterReverse() {
    return runMotors(-0.2, -0.3);
  }

  private void stopMotors() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Left NDexter Motor", leftMotor.getEncoder().getVelocity());
    Logger.recordOutput("Right NDexter Motor", rightMotor.getEncoder().getVelocity());
  }
}
