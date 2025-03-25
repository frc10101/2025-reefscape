// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final SparkMax m_motorLeft;
  private final SparkMax m_motorRight;
  private final SysIdRoutine sysIdRoutine;

  public Elevator() {
    m_motorLeft = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorLeft, false);
    m_motorRight = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorRight, true);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVolts(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by URCL
                this));
  }

  private SparkMax configureMotor(int canID, boolean isFollower) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder.positionConversionFactor(
        2 * Math.PI / Constants.ElevatorConstants.ElevatorGearRatio);
    config.closedLoop.pidf(
        ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF);

    SparkMax motor = new SparkMax(canID, MotorType.kBrushless);
    if (isFollower) {
      config.follow(m_motorLeft, true);
      config.limitSwitch.forwardLimitSwitchEnabled(true).forwardLimitSwitchType(Type.kNormallyOpen);
    } else {
      config.inverted(true);
    }
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private void goToGoal(double goal) {
    m_motorLeft.getClosedLoopController().setReference(goal, ControlType.kPosition);
  }

  private void setElevatorSpeed(double speed) {
    m_motorLeft.set(speed);
  }

  public Command moveToPosition(double position) {
    return runOnce(() -> goToGoal(position));
  }

  public Command raise() {
    return runEnd(() -> setElevatorSpeed(0.5), () -> setElevatorSpeed(0));
  }

  public Command lower() {
    return runEnd(() -> setElevatorSpeed(-0.5), () -> setElevatorSpeed(0));
  }

  public Command stop() {
    return runOnce(
        () -> {
          m_motorLeft.set(0);
          m_motorRight.set(0);
        });
  }

  public Trigger elevatorLimit() {
    return new Trigger(m_motorRight.getForwardLimitSwitch()::isPressed);
  }

  public Command L1() {
    return moveToPosition(Constants.ElevatorConstants.L1);
  }

  public Command L2() {
    return moveToPosition(Constants.ElevatorConstants.L2);
  }

  public Command L3() {
    return moveToPosition(Constants.ElevatorConstants.L3);
  }

  public Command L4() {
    return moveToPosition(Constants.ElevatorConstants.L4);
  }

  public Command HumanPlayer() {
    return moveToPosition(Constants.ElevatorConstants.HumanPlayer);
  }

  public Command sysIdQuasistaticForward() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuasistaticReverse() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicReverse() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  private void runVolts(double volts) {
    m_motorLeft.setVoltage(volts);
    m_motorRight.setVoltage(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
