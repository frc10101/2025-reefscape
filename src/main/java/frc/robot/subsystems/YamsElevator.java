// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class YamsElevator extends SubsystemBase {
  private final SparkMax motorLeft = new SparkMax(Constants.SparkMaxCanIDs.ElevatorMotorLeft, MotorType.kBrushless);
  private final SparkMax motorRight = new SparkMax(Constants.SparkMaxCanIDs.ElevatorMotorRight, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(Meters.of(2 * Math.PI * ElevatorConstants.kElevatorDrumRadius))
      .withClosedLoopController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD)
      .withSimClosedLoopController(6.189, 0, 0)
      .withFeedforward(new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV))
      .withSimFeedforward(new ElevatorFeedforward(0, 0.04, 38.19,0.0))
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
      .withGearing(ElevatorConstants.ElevatorGearRatio)
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(ElevatorConstants.kCurrentLimit))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withFollowers(Pair.of(motorRight, true));

  private final SmartMotorController smartMotorController = new SparkWrapper(motorLeft, DCMotor.getNEO(2), motorConfig);

  private final ElevatorConfig elevatorConfig = new ElevatorConfig(smartMotorController)
      .withStartingHeight(Meters.of(ElevatorConstants.kMinElevatorHeightMeters))
      .withHardLimits(
          Meters.of(ElevatorConstants.kMinElevatorHeightMeters),
          Meters.of(ElevatorConstants.kMaxElevatorHeightMeters))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(ElevatorConstants.kCarriageMass));

  private final Elevator elevator = new Elevator(elevatorConfig);

  public YamsElevator() {}

  public Command moveToPosition(double position) {
    return elevator.setHeight(Meters.of(position));
  }

  public Command raise() {
    return elevator.set(0.5);
  }

  public Command lower() {
    return elevator.set(-0.5);
  }

  public Command stop() {
    return elevator.set(0);
  }

  public Command L1() {
    return moveToPosition(ElevatorConstants.L1);
  }

  public Command L2() {
    return moveToPosition(ElevatorConstants.L2);
  }

  public Command L3() {
    return moveToPosition(ElevatorConstants.L3);
  }

  public Command HumanPlayer() {
    return moveToPosition(ElevatorConstants.HumanPlayer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }

}