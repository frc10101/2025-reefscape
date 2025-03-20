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
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.SparkMaxCanIDs;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ICEE extends SubsystemBase {
  private final SparkMax motor;

  public ICEE() {
    motor = new SparkMax(SparkMaxCanIDs.IceeMotor, MotorType.kBrushless);
    configureMotor();
  }

  private void configureMotor() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.encoder.velocityConversionFactor(Constants.IceeConstants.ratio);
    motorConfig.limitSwitch.forwardLimitSwitchEnabled(true).forwardLimitSwitchType(Type.kNormallyOpen);
    motorConfig.smartCurrentLimit(50);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.IceeConstants.Kp)
        .i(Constants.IceeConstants.Ki)
        .d(Constants.IceeConstants.Kd)
        .velocityFF(Constants.IceeConstants.FF)
        .outputRange(-1, 1);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public Trigger ICEELimit() {
    return new Trigger(motor.getForwardLimitSwitch()::isPressed);
  }

  public BooleanSupplier getLimitSwitch() {
    return motor.getForwardLimitSwitch()::isPressed;
  }

  public Command runOut() {
    return setMotorVelocity(Constants.IceeConstants.IceeOutVelocity);
  }

  public Command runIn() {
    return setMotorVelocity(
        motor.getForwardLimitSwitch().isPressed() ? 0 : Constants.IceeConstants.IceeInVelocity);
  }

  private Command setMotorVelocity(double velocity) {
    return runOnce(() -> motor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0));
  }

  public Command Intake() {
    return runMotor(0.5);
  }

  public Command spitOut() {
    return runMotor(-0.75);
  }

  private Command runMotor(double speed) {
    return runEnd(() -> motor.set(speed), () -> motor.set(0));
  }

  public Command stop() {
    return runOnce(() -> motor.set(0));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("ICEE motor", motor.getEncoder().getVelocity());
    Logger.recordOutput("ICEE Limit Switch", motor.getForwardLimitSwitch().isPressed());
  }
}
