package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final TalonFX armMotor;
  private Double kF = Constants.ArmConstants.kFF;
  private PositionVoltage pidControl = null;

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
    Slot0Configs config = new Slot0Configs();
    config.kP = Constants.ArmConstants.kP;
    config.kI = Constants.ArmConstants.kI;
    config.kD = Constants.ArmConstants.kD;
    armMotor.getConfigurator().apply(config);
    this.pidControl = new PositionVoltage(0).withSlot(0);
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

  public Command coralFF() {
    return runOnce(
        () -> {
          this.kF = Constants.ArmConstants.kFFwithCoral;
        });
  }

  public Command normalFF() {
    return runOnce(
        () -> {
          this.kF = Constants.ArmConstants.kFF;
        });
  }

  public Command setArmPosition(double position) {
    return new Command() {
      private double targetPosition;

      @Override
      public void initialize() {
        targetPosition = position;
        armMotor.setControl(pidControl.withPosition(targetPosition).withFeedForward(kF));
      }

      @Override
      public boolean isFinished() {
        double currentPosition = armMotor.getPosition().getValueAsDouble();
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
