package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final SparkMax armMotor;

  public Arm() {
    armMotor = new SparkMax(Constants.SparkMaxCanIDs.StrawPivotMotor, MotorType.kBrushless);
    armMotor.getEncoder().setPosition(Math.PI);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.closedLoop.pidf(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kFF);

    armConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(ArmConstants.kMaxAcceleration)
        .maxVelocity(ArmConstants.kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    armConfig.inverted(true);
    armConfig.encoder.positionConversionFactor(2.0 * Math.PI / ArmConstants.GEAR_RATIO);
    armConfig.idleMode(IdleMode.kBrake);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armMotor.getEncoder().setPosition(Math.PI);
  }

  public double getAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public void stop() {
    armMotor.set(0);
  }

  public void moveArm(double goal) { // need voltage power from 1-0(duty cycle) * 13
    double angle = armMotor.getEncoder().getPosition();
    double compensation =
        angle < 0
            ? 0
            : (armMotor.configAccessor.closedLoop.getFF() * Math.sin(angle) + angle
                    > Math.toRadians(167)
                ? .5
                : 0);
    armMotor.setVoltage(goal + compensation);
  }

  @Override
  public void periodic() {
    // No continuous updates needed
    // System.out.println(armMotor.getEncoder().getPosition() * 180 / Math.PI);
  }

  public Command armUp() {
    return runEnd(() -> moveArm(-3.0), () -> moveArm(0));
  }

  public Command armDown() {
    return runEnd(() -> moveArm(3.0), () -> moveArm(0));
  }

  public Command stopArm() {
    return run(
        () -> {
          double angle = armMotor.getEncoder().getPosition();
          double compensation =
              angle < 0 ? 0 : (armMotor.configAccessor.closedLoop.getFF() * Math.sin(angle));
          armMotor.setVoltage(0 + compensation);
        });
  }

  public Command coralFF() {
    return runOnce(
        () -> {
          SparkMaxConfig armConfig = new SparkMaxConfig();
          armConfig.closedLoop.pidf(
              ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kFFwithCoral);

          armConfig
              .closedLoop
              .maxMotion
              .maxAcceleration(ArmConstants.kMaxAcceleration)
              .maxVelocity(ArmConstants.kMaxVelocity)
              .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
          armConfig.inverted(true);
          armConfig.encoder.positionConversionFactor(2.0 * Math.PI / ArmConstants.GEAR_RATIO);
          armConfig.idleMode(IdleMode.kBrake);

          armMotor.configure(
              armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
  }

  public Command normalFF() {
    return runOnce(
        () -> {
          SparkMaxConfig armConfig = new SparkMaxConfig();
          armConfig.closedLoop.pidf(
              ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, ArmConstants.kFF);

          armConfig.inverted(true);
          armConfig.encoder.positionConversionFactor(2.0 * Math.PI / ArmConstants.GEAR_RATIO);
          armConfig.idleMode(IdleMode.kBrake);

          armMotor.configure(
              armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
  }

  public Command setArmPosition(double position) {
    return new Command() {
      private double targetPosition;

      @Override
      public void initialize() {
        targetPosition = position;
        armMotor
            .getClosedLoopController()
            .setReference(position, ControlType.kMAXMotionPositionControl);
      }

      @Override
      public boolean isFinished() {
        double currentPosition = armMotor.getEncoder().getPosition();
        return Math.abs(currentPosition - targetPosition) < 0.05;
      }

      @Override
      public void end(boolean interrupted) {
        if (interrupted) {
          stop();
        }
      }
    }.withName("SetArmPosition");
  }
}
