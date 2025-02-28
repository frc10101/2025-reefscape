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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.SparkMaxCan;

public class ICEE extends SubsystemBase {

  private final SparkMax motor;

  /** Creates a new ICEE. */
  public ICEE() {

    this.motor = new SparkMax(SparkMaxCan.ICEEID, MotorType.kBrushless);
    SparkMaxConfig motorConfig= new SparkMaxConfig();

    motorConfig.encoder.positionConversionFactor(Constants.IceeConstants.ratio).velocityConversionFactor(Constants.IceeConstants.ratio);


    motorConfig
        .limitSwitch
        .forwardLimitSwitchEnabled(true);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.IceeConstants.Kp, ClosedLoopSlot.kSlot0)
        .i(Constants.IceeConstants.Ki, ClosedLoopSlot.kSlot0)
        .d(Constants.IceeConstants.Kd, ClosedLoopSlot.kSlot0)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }


  public Trigger ICEELimit() {
    return new Trigger(this.motor.getForwardLimitSwitch()::isPressed);
  }

  public Command runOut() {

    return runOnce(
        () -> {
          motor.getClosedLoopController().setReference(
              Constants.IceeConstants.IceeOutVelocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        });
  }

  public Command runIn() {
    return runOnce(
        () -> {
          motor.getClosedLoopController().setReference(
              motor.getForwardLimitSwitch().isPressed()?0:Constants.IceeConstants.IceeInVelocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
