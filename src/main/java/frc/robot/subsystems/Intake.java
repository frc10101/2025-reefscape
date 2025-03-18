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

// import for Logger

public class Intake extends SubsystemBase {
  private final SparkMax m_motorLeft;
  private final SparkMax m_motorRotateLeft;
  private final SparkMax m_motorRight;
  private final SparkMax m_motorRotateRight;

  /** Creates a new intake. */
  public Intake() {
    m_motorLeft = new SparkMax(Constants.SparkMaxCanIDs.intakeMotorSpinLeft, MotorType.kBrushless);
    m_motorRight =
        new SparkMax(Constants.SparkMaxCanIDs.intakeMotorSpinRight, MotorType.kBrushless);
    m_motorRotateLeft =
        new SparkMax(Constants.SparkMaxCanIDs.intakeMotorPivotLeft, MotorType.kBrushless);
    m_motorRotateRight =
        new SparkMax(Constants.SparkMaxCanIDs.intakeMotorPivotRight, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig configRotate = new SparkMaxConfig();
    SparkMaxConfig configFollower = new SparkMaxConfig();
    SparkMaxConfig configRotateFollower = new SparkMaxConfig();

    m_motorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motorRotateLeft.configure(
        configRotate, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configFollower.follow(m_motorLeft, true);
    configRotateFollower.follow(m_motorRotateLeft, true);
    m_motorRight.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motorRotateRight.configure(
        configRotateFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void intakeUp(double goal) {
    m_motorRotateLeft.set(goal);
  }

  private void intake() {
    m_motorLeft.set(0.7);
  }

  private void spitOut() {
    m_motorLeft.set(-0.7);
  }

  @Override
  public void periodic() {}

  public Command StartIntake() {
    return runEnd(() -> intake(), () -> stop());
  }

  public Command StartSpitout() {
    return runEnd(() -> spitOut(), () -> stop());
  }

  public Command IntakeUp() {
    return runEnd(() -> intakeUp(-.2), () -> intakeUp(0));
  }

  public Command IntakeDown() {
    return runEnd(() -> intakeUp(.2), () -> intakeUp(0));
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorLeft.set(0);
        });
  }
}
