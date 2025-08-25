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

public class Elevator extends SubsystemBase {
  private final SparkMax m_motorLeft;
  private final SparkMax m_motorRight;
  private final SparkMaxSim FakeBoi;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 14, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              Constants.ElevatorConstants.kMinElevatorHeightMeters
                  * Constants.ElevatorConstants.kPixelsPerMeter,
              90));

  public Elevator() {
    m_motorLeft = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorLeft, false);
    m_motorRight = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorRight, true);
    FakeBoi = new SparkMaxSim(m_motorLeft, DCMotor.getNEO(1));
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

  public Command HumanPlayer() {
    return moveToPosition(Constants.ElevatorConstants.HumanPlayer);
  }

  @Override
  public void periodic() {
    moveToPosition(Constants.ElevatorConstants.L1);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Applied output", FakeBoi.getAppliedOutput());  // Shows applied output is changing when a button is pressed so we know atleast something is working
    SmartDashboard.putNumber("Line Length", m_elevatorMech2d.getLength());  // The length of the line simulating our elevator moving in pixels (bottom to top is the distance travelled)
    SmartDashboard.putNumber("Elevator Height", m_elevatorMech2d.getLength()/ Constants.ElevatorConstants.kPixelsPerMeter); // The height of the elevator in meters

    // Update mechanism2d
    m_elevatorMech2d.setLength(
        Constants.ElevatorConstants.kPixelsPerMeter * Constants.ElevatorConstants.kMinElevatorHeightMeters
            + Constants.ElevatorConstants.kPixelsPerMeter
                * (Math.abs(m_motorLeft.getEncoder().getPosition()) / Constants.ElevatorConstants.kElevatorGearing)
                * (Constants.ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI));
  }  
  // Fake elevator
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          Constants.ElevatorConstants.kElevatorGearing,
          Constants.ElevatorConstants.kCarriageMass,
          Constants.ElevatorConstants.kElevatorDrumRadius,
          Constants.ElevatorConstants.kMinElevatorHeightMeters,
          Constants.ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0);


  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    //FakeBoi.iterate(1000, 12, 0.020);
    m_elevatorSim.setInput(FakeBoi.getAppliedOutput() * RobotController.getBatteryVoltage());

    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    FakeBoi.iterate(
      ((m_elevatorSim.getVelocityMetersPerSecond()
                  / (Constants.ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI))
              * Constants.ElevatorConstants.kElevatorGearing)
          * 60.0,
          RobotController.getBatteryVoltage(),
      0.02);

      //FakeBoi.setPosition(FakeBoi.getAppliedOutput()); - Made sure setPosition did what it was saying it was gonna
  }
}
