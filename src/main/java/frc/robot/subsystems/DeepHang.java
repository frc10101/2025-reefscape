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

public class DeepHang extends SubsystemBase {
  private final SparkMax motor;
  private double kF;

  public DeepHang() {
    motor = new SparkMax(SparkMaxCanIDs.IceeMotor, MotorType.kBrushless);
    configureMotor();
    kF = Constants.DeepHangConstants.kFUnloaded;
  }

  private void configureMotor() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.encoder.positionConversionFactor(Constants.DeepHangConstants.GEAR_RATIO);
    motorConfig.smartCurrentLimit(90);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            Constants.DeepHangConstants.kP,
            Constants.DeepHangConstants.kI,
            Constants.DeepHangConstants.kD,
            Constants.DeepHangConstants.kFUnloaded);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Command runOut() {
    return setMotorPos(Constants.DeepHangConstants.armOut);
  }

  public Command runIn() {
    return setMotorPos(Constants.DeepHangConstants.armIn);
  }

  public Command SwapKF() {

    return runOnce(
        () -> {
          kF =
              (kF == Constants.DeepHangConstants.kFLoaded
                  ? Constants.DeepHangConstants.kFUnloaded
                  : Constants.DeepHangConstants.kFLoaded);
          SparkMaxConfig motorConfig = new SparkMaxConfig();
          motorConfig
              .closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pidf(
                  Constants.DeepHangConstants.kP,
                  Constants.DeepHangConstants.kI,
                  Constants.DeepHangConstants.kD,
                  kF);
        });
  }

  private Command setMotorPos(double pos) {
    return runOnce(
        () ->
            motor
                .getClosedLoopController()
                .setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0));
  }

  public Command hangDown() {
    return runOnce(
        () ->
            motor
                .set(.5));
  }
  public Command hangUp() {
    return runOnce(
        () ->
            motor
                .set(-.5));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hang motor", motor.getEncoder().getPosition());
  }
}
