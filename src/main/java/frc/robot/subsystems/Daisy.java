// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Daisy extends SubsystemBase {
  private final SparkMax m_daisyMotor;
  private final DigitalInput m_daisyBeamBreak;

  /** Creates a new Daisy. */
  public Daisy() {
    m_daisyMotor = configureMotor(Constants.SparkMaxCanIDs.ElevatorMotorLeft);
    m_daisyBeamBreak = new DigitalInput(Constants.digitalIDs.daisyBeamBreak);
  }

  private SparkMax configureMotor(int canID) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder.positionConversionFactor(
        2 * Math.PI / Constants.ElevatorConstants.ElevatorGearRatio);
    config.closedLoop.pidf(
        ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF);

    SparkMax motor = new SparkMax(canID, MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return motor;
  }

  public Command outputSpin(double outputSpeed) {
    return runOnce(() -> m_daisyMotor.set(outputSpeed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_daisyBeamBreak.get()) {
      m_daisyMotor.set(0);
    }
  }
}
