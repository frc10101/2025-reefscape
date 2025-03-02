// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger; // import for Logger

public class Intake extends SubsystemBase {
  private final SparkMax m_motorLeft;
  // private final SparkMax m_motorRotateLeft;
  private final SparkMax m_motorRight;
  // private final SparkMax m_motorRotateRight;
  private double m_motorTargetPos = 0, m_motorRotateTarget = 0;

  /** Creates a new intake. */
  public Intake() {
    m_motorLeft = new SparkMax(Constants.SparkMaxCanIDs.intakeMotorSpinLeft, MotorType.kBrushless);
    m_motorRight =
        new SparkMax(Constants.SparkMaxCanIDs.intakeMotorSpinRight, MotorType.kBrushless);
    // m_motorRotateLeft =
    //     new SparkMax(Constants.SparkMaxCanIDs.intakeMotorPivotLeft, MotorType.kBrushless);
    // m_motorRotateRight =
    // new SparkMax(Constants.SparkMaxCanIDs.intakeMotorPivotRight, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    // SparkMaxConfig configRotate = new SparkMaxConfig();
    SparkMaxConfig configFollower = new SparkMaxConfig();
    // SparkMaxConfig configRotateFollower = new SparkMaxConfig();
    // config.closedLoop.pidf(
    //     Constants.IntakeConstants.kP,
    //     Constants.IntakeConstants.kI,
    //     Constants.IntakeConstants.kD,
    //     Constants.IntakeConstants.kFF);

    // config
    //     .closedLoop
    //     .maxMotion
    //     .maxAcceleration(Constants.IntakeConstants.kMaxAcceleration)
    //     .maxVelocity(Constants.IntakeConstants.kMaxVelocity)
    //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    m_motorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // configRotate.closedLoop.pidf(
    //     Constants.IntakeConstants.RotateKP,
    //     Constants.IntakeConstants.RotateKI,
    //     Constants.IntakeConstants.RotateKD,
    //     Constants.IntakeConstants.RotateKFF);

    // configRotate
    //     .closedLoop
    //     .maxMotion
    //     .maxAcceleration(Constants.IntakeConstants.RotateKMaxAcceleration)
    //     .maxVelocity(Constants.IntakeConstants.RotateKMaxVelocity)
    //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    // configRotate.encoder.positionConversionFactor(
    //     2.0 * Math.PI * Constants.IntakeConstants.IntakeGearRatio);

    configFollower.follow(m_motorLeft, true);

    m_motorRight.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // configRotateFollower.follow(m_motorRotateLeft, true);

    // m_motorRotateRight.configure(
    //     configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // private void goToGoal(double goal) {
  //   m_motorRotateTarget = goal;
  //   m_motorRotateLeft
  //       .getClosedLoopController()
  //       .setReference(goal, ControlType.kMAXMotionPositionControl);
  // }

  // private void intakeUp() {
  //   goToGoal(Constants.IntakeConstants.IntakeUpPos);
  // }

  // private void intakeDown() {
  //   goToGoal(Constants.IntakeConstants.IntakeDownPos);
  // }

  private void intake() {
    m_motorLeft.set(0.7);
    // m_motorTargetPos = 1;
    // m_motorLeft.getClosedLoopController().setReference(1, ControlType.kMAXMotionVelocityControl);
  }

  private void spitOut() {
    m_motorLeft.set(-0.7);
    // m_motorTargetPos = -1;
    // m_motorLeft.getClosedLoopController().setReference(-1,
    // ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "M_MotorRotate Error", m_motorTargetPos - m_motorLeft.getEncoder().getPosition());
    // Logger.recordOutput(
    //     "M_MotorRotate Error", m_motorRotateTarget -
    // m_motorRotateLeft.getEncoder().getPosition());
  }

  public Command StartIntake() {
    return runEnd(() -> intake(), () -> m_motorLeft.stopMotor());
  }

  public Command StartSpitout() {
    return runEnd(() -> spitOut(), () -> m_motorLeft.stopMotor());
  }

  public Command stop() {
    return runOnce(() -> m_motorLeft.set(0));
  }

  // public Command IntakeUp() {
  //   return runOnce(() -> intakeUp());
  // }

  // public Command IntakeDown() {
  //   return runOnce(() -> intakeDown());
  // }
}
