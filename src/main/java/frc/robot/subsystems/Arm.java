// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private SparkMax m_armMotor;
  private final double kMaxVelocity = 2.0;
  private final double kMaxAcceleration = 1.0;
  private final double kP = 1.0;
  private final double kI = 0.0;
  private final double kD = 0.5;
  private final double kFF = 0.8;

  /** Creates a new arm. */
  public Arm() {
    m_armMotor = new SparkMax(2, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.pidf(kP, kI, kD, kFF);

    config
        .closedLoop
        .maxMotion
        .maxAcceleration(kMaxAcceleration)
        .maxVelocity(kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    config.encoder.positionConversionFactor(2.0 * Math.PI * 1.0);

    m_armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.closedLoop.pidf(kP, kI, kD, kFF);
  }

  private void moveToPosition(double position) {
    m_armMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command armToPosition(double position) {
    return run(() -> moveToPosition(position));
  }
}
