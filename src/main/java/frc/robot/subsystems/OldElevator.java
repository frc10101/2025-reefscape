// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class OldElevator extends SubsystemBase {
  private final SparkMax m_motorLeft;
  private final SparkMax m_motorRight;
  private double position;
  private double velocity;

  // Simulation setup and variables
  private DCMotor elevatorMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          Constants.ElevatorConstants.ElevatorGearRatio,
          Constants.ElevatorConstants.kCarriageMass,
          Constants.ElevatorConstants.kElevatorDrumRadius,
          Constants.ElevatorConstants.kMinElevatorHeightMeters,
          Constants.ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          Constants.ElevatorConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 50);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              Constants.ElevatorConstants.kMinElevatorHeightMeters
                  * Constants.ElevatorConstants.kPixelsPerMeter,
              90));

  public OldElevator() {
    m_motorLeft = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorLeft, false);
    m_motorRight = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorRight, true);
    position = 0;
    velocity = 0;

    SmartDashboard.putData("Elevator Sim", m_mech2d);

    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(m_motorLeft, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(m_motorLeft, false);
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
    }
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  private void goToGoal(double goal) {
    position = goal;
  }

  private void setElevatorSpeed(double speed) {
    velocity = speed;
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

  public Command HumanPlayer() {
    return moveToPosition(Constants.ElevatorConstants.HumanPlayer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motorLeft.getClosedLoopController().setReference(position, ControlType.kPosition);
    m_motorLeft.set(velocity);

    // Log motor applied output (what percent it’s actually doing)
    Logger.recordOutput("Elevator/MotorOutput", m_motorLeft.getAppliedOutput());
    // Log encoder position
    Logger.recordOutput("Elevator/Position", position);
    // Log encoder velocity
    Logger.recordOutput("Elevator/Velocity", m_motorLeft.getAbsoluteEncoder().getVelocity());
    // Log encoder speed
    Logger.recordOutput("Elevator/Speed", velocity);

    // SmartDashboard.putNumber("Elevator Position", position);
    // SmartDashboard.putNumber("Elevator Sim Position", m_elevatorSim.getPositionMeters());
    // SmartDashboard.putNumber("Mech2d Inches",
    // Units.metersToInches(Math.abs(m_elevatorMech2d.getLength())/Constants.ElevatorConstants.kPixelsPerMeter));
    // SmartDashboard.putNumber("Carriage Height",
    // (Units.metersToInches(Math.abs(m_elevatorMech2d.getLength())/Constants.ElevatorConstants.kPixelsPerMeter) + 20.5));
    SmartDashboard.putNumber(
        "Daisy Height",
        (Units.metersToInches(
                Math.abs(m_elevatorMech2d.getLength())
                    / Constants.ElevatorConstants.kPixelsPerMeter)
            + 29));

    // Update mechanism2d
    m_elevatorMech2d.setLength(
        Constants.ElevatorConstants.kPixelsPerMeter
                * Constants.ElevatorConstants.kMinElevatorHeightMeters
            + Constants.ElevatorConstants.kPixelsPerMeter
                * (position / Constants.ElevatorConstants.ElevatorGearRatio)
                * (Constants.ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(
        elevatorMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (Constants.ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * Constants.ElevatorConstants.ElevatorGearRatio)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
}
