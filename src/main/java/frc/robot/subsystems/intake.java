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

public class intake extends SubsystemBase {
  private SparkMax m_motor;
  private SparkMax m_motorRotate;
  private final double kMaxVelocity = 1.75;
  private final double kMaxAcceleration = 0.75;
  private final double kP = 1.3;
  private final double kI = 0.0;
  private final double kD = 0.7;
  private final double kFF = 1.1;

  private final double RotateKMaxVelocity = 1.75;
  private final double RotateKMaxAcceleration = 0.75;
  private final double RotateKP = 1.3;
  private final double RotateKI = 0.0;
  private final double RotateKD = 0.7;
  private final double RotateKFF = 1.1;
  /** Creates a new intake. */
  public intake() {
    m_motor = new SparkMax(3, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig configRotate = new SparkMaxConfig();
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
        config.closedLoop
        .pidf(kP, kI, kD, kFF);

    configRotate.closedLoop.maxMotion
        .maxAcceleration(RotateKMaxAcceleration)
        .maxVelocity(RotateKMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        configRotate.encoder
        .positionConversionFactor(2.0 * Math.PI * 1.5);

    m_motorRotate.configure(
        configRotate,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        configRotate.closedLoop
        .pidf(kP, kI, kD, kFF);
  }

  private void goToGoal(double goal) {
    m_motorRotate.getClosedLoopController().setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  private void upSpool(){
    m_motor.getClosedLoopController().setReference(1, ControlType.kMAXMotionVelocityControl);
  }
  private void downSpool(){
    m_motor.getClosedLoopController().setReference(-1, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command spoolUp(){
    return run(() -> upSpool());
  }
  public Command spoolDown(){
    return run(() -> downSpool());
  }
  public Command intakeUp(){
    return run(() -> goToGoal(5));
  }
  public Command intakeDown(){
    return run(() -> goToGoal(0));
  }

}