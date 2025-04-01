package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final TalonFX armMotor;
  private double targetPosition = SmartDashboard.getNumber("targetPosition", 0);
  // private double kP = SmartDashboard.getNumber("kP", Constants.ArmConstants.kP);
  // private double kI = SmartDashboard.getNumber("kI", Constants.ArmConstants.kI);
  // private double kD = SmartDashboard.getNumber("kD", Constants.ArmConstants.kD);
  // private double kFF = SmartDashboard.getNumber("kFF", Constants.ArmConstants.kFF);

  //   private final SysIdRoutine sysid;

  public Arm() {
    armMotor = new TalonFX(Constants.SparkMaxCanIDs.StrawPivotMotor);
    this.configureArmMotor();
    //     sysid = new SysIdRoutine(
    //       new SysIdRoutine.Config(),
    //       new SysIdRoutine.Mechanism(this::voltageDrive,

    //        ,

    //        this)
    //     );
  }

  private void configureArmMotor() {
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    FeedbackConfigs feedback = new FeedbackConfigs();
    feedback.SensorToMechanismRatio = Constants.ArmConstants.GEAR_RATIO;
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;
    currentLimits.SupplyCurrentLimit = 60;
    feedback.FeedbackRotorOffset = 0.619629;
    armMotor.getConfigurator().apply(motorOutput);
    armMotor.getConfigurator().apply(feedback);
    armMotor.getConfigurator().apply(currentLimits);
  }

  @SuppressWarnings("unused")
  private Command voltageDrive(double pow) {
    return Commands.runOnce(
        () -> {
          armMotor.setVoltage(pow);
        });
  }

  public double getAngle() {
    return (armMotor.getPosition().getValueAsDouble()) * 2 * Math.PI;
  }

  public void stop() {
    armMotor.set(0);
  }

  public void moveArm(double goal) {
    double angle = armMotor.getPosition().getValueAsDouble();
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
    return armMotor.getClosedLoopFeedForward().getValueAsDouble() * Math.sin(angle);
  }

  public Command armUp() {
    return runEnd(() -> moveArm(-3.0), this::stop);
  }

  public Command armDown() {
    return runEnd(() -> moveArm(3.0), this::stop);
  }

  public Command stopArm() {
    return run(
        () -> {
          double angle = armMotor.getPosition().getValueAsDouble();
          double compensation = calculateCompensation(angle);
          armMotor.setVoltage(compensation);
        });
  }

  // public Command coralFF() {
  //   return runOnce(
  //       () -> {
  //         this.kF = Constants.ArmConstants.kFFwithCoral;
  //       });
  // }

  // public Command normalFF() {
  //   return runOnce(
  //       () -> {
  //         this.kF = Constants.ArmConstants.kFF;
  //       });
  // }

  public Command setArmPosition(double position) {
    return runOnce(
        () -> {
          targetPosition = position;
        });
  }

  private void armUpdate(double pos) {
    // Current arm position
    double currentPosition = armMotor.getPosition().getValueAsDouble();

    // Calculate error
    double error = pos - currentPosition;

    // PID terms
    double proportional = Constants.ArmConstants.kP * error;

    // I term could track accumulated error if needed
    // This is simplified; in practice you might want anti-windup protection
    double integral = Constants.ArmConstants.kI * error;

    // D term - rate of change of error
    // Using motor velocity directly instead of calculating derivative of error
    double derivative = Constants.ArmConstants.kD * -armMotor.getVelocity().getValueAsDouble();

    // G term - gravity compensation based on sin of the position
    // The sign and magnitude of this term depends on your arm's mechanics
    double gravityCompensation = Constants.ArmConstants.kFF * Math.sin(currentPosition);

    // Calculate total output
    double outputVoltage = proportional + integral + derivative + gravityCompensation;

    // Apply voltage to the motor
    armMotor.setVoltage(outputVoltage);
  }

  @Override
  public void periodic() {
    armUpdate(targetPosition);
  }
}
