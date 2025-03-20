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
    configureArmMotor();
  }

  private void configureArmMotor() {
    SparkMaxConfig armConfig = createArmConfig(ArmConstants.kFF);
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armMotor.getEncoder().setPosition(Math.PI);
  }

  private SparkMaxConfig createArmConfig(double feedForward) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, feedForward);
    config.closedLoop.maxMotion
        .maxAcceleration(ArmConstants.kMaxAcceleration)
        .maxVelocity(ArmConstants.kMaxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    config.inverted(true);
    config.encoder.positionConversionFactor(2.0 * Math.PI / ArmConstants.GEAR_RATIO);
    config.idleMode(IdleMode.kBrake);
    return config;
  }

  public double getAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public void stop() {
    armMotor.set(0);
  }

  public void moveArm(double goal) {
    double angle = armMotor.getEncoder().getPosition();
    double compensation = calculateCompensation(angle);
    armMotor.setVoltage(goal + compensation);
  }

  private double calculateCompensation(double angle) {
    if (angle < 0) {
      return 0;
    }
    if (angle > Math.toRadians(167)) {
      return 0.5;
    }
    return armMotor.configAccessor.closedLoop.getFF() * Math.sin(angle);
  }

  public Command armUp() {
    return runEnd(() -> moveArm(-3.0), this::stop);
  }

  public Command armDown() {
    return runEnd(() -> moveArm(3.0), this::stop);
  }

  public Command stopArm() {
    return run(() -> {
      double angle = armMotor.getEncoder().getPosition();
      double compensation = calculateCompensation(angle);
      armMotor.setVoltage(compensation);
    });
  }

  public Command coralFF() {
    return runOnce(() -> armMotor.configure(createArmConfig(ArmConstants.kFFwithCoral), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public Command normalFF() {
    return runOnce(() -> armMotor.configure(createArmConfig(ArmConstants.kFF), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public Command setArmPosition(double position) {
    return new Command() {
      private double targetPosition;

      @Override
      public void initialize() {
        targetPosition = position;
        armMotor.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl);
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

  @Override
  public void periodic() {
    // No continuous updates needed
  }
}
