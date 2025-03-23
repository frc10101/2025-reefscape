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
  private final SparkMax m_motorLeft;
  private final SparkMax m_motorRotateLeft;

  @SuppressWarnings("unused")
  private final SparkMax m_motorRight;

  @SuppressWarnings("unused")
  private final SparkMax m_motorRotateRight;

  public Intake() {
    m_motorLeft = configureMotor(Constants.SparkMaxCanIDs.intakeMotorSpinLeft, false);
    m_motorRight = configureMotor(Constants.SparkMaxCanIDs.intakeMotorSpinRight, true);
    m_motorRotateLeft = configureMotor(Constants.SparkMaxCanIDs.intakeMotorPivotLeft, false);
    m_motorRotateRight = configureMotor(Constants.SparkMaxCanIDs.intakeMotorPivotRight, true);
  }

  private SparkMax configureMotor(int canID, boolean isFollower) {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMax motor = new SparkMax(canID, MotorType.kBrushless);
    if (isFollower) {
      config.follow(m_motorLeft, true);
    }
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private void setMotorSpeed(SparkMax motor, double speed) {
    motor.set(speed);
  }

  public Command StartIntake() {
    return runEnd(() -> setMotorSpeed(m_motorLeft, 0.7), this::stopMotors);
  }

  public Command StartSpitout() {
    return runEnd(() -> setMotorSpeed(m_motorLeft, -0.7), this::stopMotors);
  }

  public Command IntakeUp() {
    return runEnd(
        () -> setMotorSpeed(m_motorRotateLeft, -0.2), () -> setMotorSpeed(m_motorRotateLeft, 0));
  }

  public Command IntakeDown() {
    return runEnd(
        () -> setMotorSpeed(m_motorRotateLeft, 0.2), () -> setMotorSpeed(m_motorRotateLeft, 0));
  }

  public Command stop() {
    return runOnce(this::stopMotors);
  }

  private void stopMotors() {
    setMotorSpeed(m_motorLeft, 0);
    setMotorSpeed(m_motorRotateLeft, 0);
  }

  @Override
  public void periodic() {
    // No periodic updates needed
  }
}
