package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.Simulation.MapleSimSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class DriveSim extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.002; // 2 ms
  private Notifier m_simNotifier = null;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public DriveSim(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(5, 0, 0),
              // PID constants for rotation
              new PIDConstants(5, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kBlueAlliancePerspectiveRotation
                        : kRedAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

  private void startSimThread() {
    mapleSimSwerveDrivetrain =
        new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            Kilogram.of(51.7),
            Inches.of(30),
            Inches.of(30),
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            1.2,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void resetPose(Pose2d pose) {
    if (this.mapleSimSwerveDrivetrain != null)
      mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
    Timer.delay(0.1); // wait for simulation to update
    super.resetPose(pose);
  }

  public Command getAuto(String autoName) {
    try {
      return AutoBuilder.buildAuto(autoName);
    } catch (Exception e) {
      return null;
    }
  }
}
