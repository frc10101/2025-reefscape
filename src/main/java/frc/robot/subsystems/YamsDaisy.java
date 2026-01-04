// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class YamsDaisy extends SubsystemBase {
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              Constants.DaisyConstants.kPID_P,
              Constants.DaisyConstants.kPID_I,
              Constants.DaisyConstants.kPID_D)
          .withSimClosedLoopController(
              Constants.DaisyConstants.kPID_P,
              Constants.DaisyConstants.kPID_I,
              Constants.DaisyConstants.kPID_D)
          .withFeedforward(
              new SimpleMotorFeedforward(
                  Constants.DaisyConstants.kFeedforwardS,
                  Constants.DaisyConstants.kFeedforwardV,
                  Constants.DaisyConstants.kFeedforwardA))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  Constants.DaisyConstants.kFeedforwardS,
                  Constants.DaisyConstants.kFeedforwardV,
                  Constants.DaisyConstants.kFeedforwardA))
          .withTelemetry("DaisyMotor", TelemetryVerbosity.HIGH)
          .withGearing(Constants.DaisyConstants.kGearRatio)
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(Constants.DaisyConstants.kStatorCurrentLimitAmps));

  private final SparkMax spark =
      new SparkMax(Constants.SparkMaxCanIDs.DaisyMotor, MotorType.kBrushless);

  private final SmartMotorController smartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig daisyConfig =
      new FlyWheelConfig(smartMotorController)
          .withDiameter(Meters.of(Constants.DaisyConstants.kDiameterMeters))
          .withMass(Pounds.of(Constants.DaisyConstants.kMassPounds))
          .withTelemetry("DaisyMechanism", TelemetryVerbosity.HIGH);

  private final FlyWheel daisy = new FlyWheel(daisyConfig);

  // Simulated beam break sensor
  private final DigitalInput dio = new DigitalInput(Constants.DigitalIDs.daisyBeamBreak);
  private final yams.motorcontrollers.simulation.Sensor coralSensor =
      new yams.mechanisms.config.SensorConfig("CoralDetectorBeamBreak")
          .withField("Beam", dio::get, false) // Default value is false
          .getSensor();

  public YamsDaisy() {}

  /**
   * Sets the velocity of the Daisy mechanism.
   *
   * @param velocity The desired velocity RPM.
   * @return A command to set the velocity.
   */
  public Command setVelocity(AngularVelocity velocity) {
    return daisy.setSpeed(velocity);
  }

  public double getChoralVelocity() {
    double speed = daisy.getSpeed().baseUnitMagnitude();
    String unit =
        daisy
            .getSpeed()
            .baseUnit()
            .name(); // Assuming getUnit() returns the unit as a String or Enum
    return switch (unit) {
      case "RPM" -> speed * (2 * Math.PI * (Constants.DaisyConstants.kDiameterMeters / 2)) / 60.0;
      case "Radian per Second" -> speed * (Constants.DaisyConstants.kDiameterMeters / 2);
      case "DegreesPerSecond" -> speed
          * (Math.PI / 180)
          * (Constants.DaisyConstants.kDiameterMeters / 2);
      default -> throw new IllegalArgumentException("Unsupported angular velocity unit: " + unit);
    };
  }

  /**
   * Sets the duty cycle of the Daisy mechanism.
   *
   * @param dutyCycle The desired duty cycle.
   * @return A command to set the duty cycle.
   */
  public Command setDutyCycle(double dutyCycle) {
    return daisy.set(dutyCycle);
  }

  @Override
  public void periodic() {
    daisy.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    daisy.simIterate();
    org.littletonrobotics.junction.Logger.recordOutput(
        "daisy Speed", RPM.convertFrom(daisy.getSpeed().magnitude(), RadiansPerSecond));
  }
}
