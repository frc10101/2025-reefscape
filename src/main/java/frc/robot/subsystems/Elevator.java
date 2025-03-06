// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final SparkMax m_motorLeft;
  private final SparkMax m_motorRight;

  /** follower motor */

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig configFollower = new SparkMaxConfig();

    m_motorLeft = new SparkMax(Constants.SparkMaxCanIDs.ElevatorMotorLeft, MotorType.kBrushless);
    m_motorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    configFollower.follow(m_motorLeft, true);

    m_motorRight = new SparkMax(Constants.SparkMaxCanIDs.ElevatorMotorRight, MotorType.kBrushless);
    m_motorRight.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configFollower
        .limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyOpen);
  }

  private void goToGoal(double goal) {
    m_motorLeft.getClosedLoopController().setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  private void raiseElevator(double goal) {
    m_motorLeft.set(goal);
  }

  public Command Ndexter() {
    return runOnce(() -> goToGoal(Constants.ElevatorConstants.NDexter));
  }

  public Command L1() {
    return runOnce(() -> goToGoal(Constants.ElevatorConstants.L1));
  }

  public Command L2() {
    return runOnce(() -> goToGoal(Constants.ElevatorConstants.L2));
  }

  public Command L3() {
    return runOnce(() -> goToGoal(Constants.ElevatorConstants.L3));
  }

  public Command L4() {
    return runOnce(() -> goToGoal(Constants.ElevatorConstants.L4));
  }

  public Command HumanPlayer() {
    return runOnce(() -> goToGoal(Constants.ElevatorConstants.HumanPlayer));
  }

  public Command raise() {
    return runEnd(() -> raiseElevator(.5), () -> raiseElevator(0));
  }

  public Command lower() {

    return runEnd(() -> raiseElevator(-.5), () -> raiseElevator(0));
  }

  public Command zero() {
    return runOnce(() -> raiseElevator(0));
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorLeft.set(0);
          m_motorRight.set(0);
        });
  }

  public Trigger elevatorLimit() {
    return new Trigger(this.m_motorRight.getForwardLimitSwitch()::isPressed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
