// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final double kMaxVelocity = 1.75;
  private final double kMaxAcceleration = 0.75;
  private final double kP = 1.3;
  private final double kI = 0.0;
  private final double kD = 0.7;
  private final double kFF = 1.1;

  private final SparkMax m_motor;
  private final SparkMax m_motorFollower;

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig configFollower = new SparkMaxConfig();
    config.closedLoop
        .pidf(kP, kI, kD, kFF);

    config.closedLoop.maxMotion
        .maxAcceleration(kMaxAcceleration)
        .maxVelocity(kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    config.encoder
        .positionConversionFactor(2.0 * Math.PI * 1.5);

    m_motor = new SparkMax(1, MotorType.kBrushless);
    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    configFollower.follow(m_motor,true);

    m_motorFollower = new SparkMax(2,MotorType.kBrushless);
    m_motorFollower.configure(
      configFollower,
      ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private void goToGoal(double goal) {
    m_motor.getClosedLoopController().setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  public Command raise() {
    return run(() -> goToGoal(5));
  }

  public Command lower() {
    return run(() -> goToGoal(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}