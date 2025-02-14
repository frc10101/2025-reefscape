// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkBase.ControlType;

public class Arm extends SubsystemBase {
  private final double kMaxVelocity = 1.75;
  private final double kMaxAcceleration = 0.75;
  private final double kP = 1.3;
  private final double kI = 0.0;
  private final double kD = 0.7;
  private final double kFF = 1.1;

  private SparkMax armMotor;

  private RelativeEncoder armEncoder;

  private PIDController armController;

  private double armPosition;

  private SparkMaxConfig armMotorConfig;

  private SparkClosedLoopController armClosedLoopController;

  public Arm() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
        .pidf(kP, kI, kD, kFF);
  config.closedLoop.maxMotion
        .maxAcceleration(kMaxAcceleration)
        .maxVelocity(kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

  private SparkMaxConfig armMotorConfig;

  public Arm() {
    armMotorConfig = new SparkMaxConfig();
    armMotorConfig
        .closedLoop
        .pidf(kP, kI, kD, kFF)
        .maxMotion
        .maxAcceleration(kMaxAcceleration)
        .maxVelocity(kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

  config.encoder
        .positionConversionFactor(2.0 * Math.PI * 1.5);

  armMotor = new SparkMax(1, MotorType.kBrushless);
  armMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    armMotorConfig.encoder.positionConversionFactor(2.0 * Math.PI * 1.5);

    armMotor = new SparkMax(1, MotorType.kBrushless);
    armMotor.configure(
        armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void goToGoal(double goal) {
    armMotor.getClosedLoopController().setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  public Command move(double goal) {
    return run(() -> goToGoal(goal));
  }

  @Override
  public void periodic() {}
}
