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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SparkMaxCan;

public class ICEE extends SubsystemBase {

  private final SparkMax motor;

  private double targetVelocity;

  private final SparkMaxConfig motorConfig;

  private final SparkClosedLoopController closedLoopController;

  private final RelativeEncoder encoder;

  /** Creates a new ICEE. */
  public ICEE() {

    this.motor = new SparkMax(SparkMaxCan.ICEEID, MotorType.kBrushless);
    this.targetVelocity = 0;

    this.closedLoopController = this.motor.getClosedLoopController();

    this.encoder = motor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    motorConfig
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

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setTargetVelocity(double leftTarget, double rightTarget) {
    this.targetVelocity = leftTarget;
  }

  public Command runOnPIDIn() {

    return runOnce(
        () -> {
          if (!this.getSwitchState()) {
            closedLoopController.setReference(
                this.targetVelocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
          } else {
            closedLoopController.setReference(
                0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
          }
        });
  }

  public Trigger ICEELimit() {
    return new Trigger(this.motor.getForwardLimitSwitch()::isPressed);
  }

  public Command runOnPIDOut() {

    return runOnce(
        () -> {
          closedLoopController.setReference(
              -this.targetVelocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        });
  }

  public boolean getSwitchState() {
    return this.motor.getForwardLimitSwitch().isPressed();
  }

  public double getTargetVelocitie() {
    return this.targetVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
