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
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger; // import for Logger

public class Intake extends SubsystemBase {
  private final SparkMax m_motor;
  private final SparkMax m_motorRotate;
  private final SparkMax m_motorFollower;
  private final double kMaxVelocity = 1.75;
  private final double kMaxAcceleration = 0.75;
  private final double RotateKMaxVelocity = 1.75;
  private final double RotateKMaxAcceleration = 0.75;
  private final double m_motorTargetPos = 0, m_motorRotateTarget = 0;

  /** Creates a new intake. */
  public Intake() {
    m_motor = new SparkMax(Constants.canIDs.intakeMotor, MotorType.kBrushless);
    m_motorFollower = new SparkMax(Constants.canIDs.intakeMotorFollower, MotorType.kBrushless);
    m_motorRotate = new SparkMax(Constants.canIDs.intakeMotorRotate, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig configRotate = new SparkMaxConfig();
    SparkMaxConfig configFollower = new SparkMaxConfig();
    config.closedLoop.pidf(
        Constants.intakeConstants.kP,
        Constants.intakeConstants.kI,
        Constants.intakeConstants.kD,
        Constants.intakeConstants.kFF);

    config
        .closedLoop
        .maxMotion
        .maxAcceleration(kMaxAcceleration)
        .maxVelocity(kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    config.encoder.positionConversionFactor(2.0 * Math.PI * 1.5);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configRotate.closedLoop.pidf(
        Constants.intakeConstants.RotateKP,
        Constants.intakeConstants.RotateKI,
        Constants.intakeConstants.RotateKD,
        Constants.intakeConstants.RotateKFF);

    configRotate
        .closedLoop
        .maxMotion
        .maxAcceleration(RotateKMaxAcceleration)
        .maxVelocity(RotateKMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    configRotate.encoder.positionConversionFactor(2.0 * Math.PI * 1.5);

    m_motorRotate.configure(
        configRotate, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configFollower.follow(m_motor, true);

    m_motorFollower.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void goToGoal(double goal) {
    m_motorTargetPos = goal;
    m_motorRotate
        .getClosedLoopController()
        .setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  private void upSpool() {
    m_motorRotateTarget = 1;
    m_motor.getClosedLoopController().setReference(1, ControlType.kMAXMotionVelocityControl);
  }

  private void downSpool() {
    m_motorRotateTarget = -1;
    m_motor.getClosedLoopController().setReference(-1, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("M_Motor Error", m_motorTargetPos - m_motor.getEncoder().getPosition());
    Logger.recordOutput(
        "M_MotorRotate Error", m_motorRotateTarget - m_motorRotate.getEncoder().getPosition());
  }

  public Command spoolUp() {
    return runOnce(() -> upSpool());
  }
}
