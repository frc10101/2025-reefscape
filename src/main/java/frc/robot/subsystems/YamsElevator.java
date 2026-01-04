package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.*;
import yams.motorcontrollers.local.SparkWrapper;

public class YamsElevator extends SubsystemBase {
  private final SparkMax motorLeft =
      new SparkMax(Constants.SparkMaxCanIDs.ElevatorMotorLeft, MotorType.kBrushless);
  private final SparkMax motorRight =
      new SparkMax(Constants.SparkMaxCanIDs.ElevatorMotorRight, MotorType.kBrushless);

  private Pose3d DaisyPose;
  private Pose3d ElevatorPose;

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
          .withMechanismCircumference(
              Meters.of(2 * Math.PI * ElevatorConstants.kElevatorDrumRadius))
          .withClosedLoopController(
              ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
          .withSimClosedLoopController(
              ElevatorConstants.kSimClosedLoopP,
              ElevatorConstants.kSimClosedLoopI,
              ElevatorConstants.kSimClosedLoopD)
          .withFeedforward(
              new ElevatorFeedforward(
                  ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV))
          .withSimFeedforward(
              new ElevatorFeedforward(
                  ElevatorConstants.kSimFeedforwardS,
                  ElevatorConstants.kSimFeedforwardG,
                  ElevatorConstants.kSimFeedforwardV,
                  ElevatorConstants.kSimFeedforwardA))
          .withTelemetry("ElevatorMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withGearing(ElevatorConstants.ElevatorGearRatio)
          .withMotorInverted(false)
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(ElevatorConstants.kCurrentLimit))
          .withClosedLoopRampRate(Seconds.of(ElevatorConstants.kClosedLoopRampRate))
          .withFollowers(Pair.of(motorRight, true));

  private final SmartMotorController smartMotorController =
      new SparkWrapper(motorLeft, DCMotor.getNEO(2), motorConfig);

  private final ElevatorConfig elevatorConfig =
      new ElevatorConfig(smartMotorController)
          .withStartingHeight(Meters.of(ElevatorConstants.kMinElevatorHeightMeters))
          .withHardLimits(
              Meters.of(ElevatorConstants.kMinElevatorHeightMeters),
              Meters.of(ElevatorConstants.kMaxElevatorHeightMeters))
          .withTelemetry("Elevator", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(ElevatorConstants.kCarriageMass));

  private final Elevator elevator = new Elevator(elevatorConfig);

  public YamsElevator() {}

  public Command moveToPosition(double position) {
    return elevator.setHeight(Meters.of(position));
  }

  public Command raise() {
    return elevator.set(ElevatorConstants.kRaiseSpeed);
  }

  public Command lower() {
    return elevator.set(ElevatorConstants.kLowerSpeed);
  }

  public Command stop() {
    return elevator.set(ElevatorConstants.kStopSpeed);
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
    elevator.updateTelemetry();
  }

  private void ejectCoralInternal(SwerveDriveSimulation driveSim, YamsDaisy daisy) {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeCoralOnFly(
                driveSim.getSimulatedDriveTrainPose().getTranslation(),
                new Translation2d(Units.inchesToMeters(9.755), Units.inchesToMeters(3)),
                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                driveSim.getSimulatedDriveTrainPose().getRotation().plus(Rotation2d.fromDegrees(180)),
                Meters.of(DaisyPose.getZ() + ElevatorConstants.kEjectHeightAboveDaisy),
                MetersPerSecond.of(daisy.getChoralVelocity()),
                Degrees.of(ElevatorConstants.kEjectAngle)));
  }
//SIMULATION ONLY
  public Command ejectCoral(SwerveDriveSimulation drive, YamsDaisy daisy) {
    return runOnce(() -> ejectCoralInternal(drive, daisy));
  }

  @Override
  public void simulationPeriodic() {
    elevator.simIterate();
    Translation3d pose = elevator.getRelativeMechanismPosition();
    ElevatorPose = new Pose3d(0, pose.getY(), -pose.getZ(), new Rotation3d());
    DaisyPose = new Pose3d(0, pose.getY(), -pose.getZ(), new Rotation3d());

    if (-pose.getZ() > ElevatorConstants.kMaxElevatorPoseHeightMeters) {
      ElevatorPose =
          new Pose3d(
              0, pose.getY(), ElevatorConstants.kMaxElevatorPoseHeightMeters, new Rotation3d());
    }
    if (-pose.getZ() > ElevatorConstants.kMaxDaisyPoseHeightMeters) {
      DaisyPose =
          new Pose3d(0, pose.getY(), ElevatorConstants.kMaxDaisyPoseHeightMeters, new Rotation3d());
    }

    Logger.recordOutput("ElevatorPose", ElevatorPose);
    Logger.recordOutput("DaisyPose", DaisyPose);
  }
}
