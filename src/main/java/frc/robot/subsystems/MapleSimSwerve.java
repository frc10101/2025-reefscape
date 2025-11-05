package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Consumer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;

public class MapleSimSwerve implements SwerveDrive{

    private final Field2d field;
    private final GyroIO gyroIO;
    //private final Consumer<Pose2d> resetSimulationPose;

    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;
    

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
                
              private static final double ROBOT_MASS_KG = 51.7;
              private static final double ROBOT_MOI = 6.883;
              private static final double WHEEL_COF = 1.2;
              private static final RobotConfig PP_CONFIG =
                  new RobotConfig(
                      ROBOT_MASS_KG,
                      ROBOT_MOI,
                      new ModuleConfig(
                          TunerConstants.FrontLeft.WheelRadius,
                          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                          WHEEL_COF,
                          DCMotor.getKrakenX60Foc(1)
                              .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                          TunerConstants.FrontLeft.SlipCurrent,
                          1),
                      getModuleTranslations());
            
                public MapleSimSwerve(
                    GyroIO gyroIO,
                ModuleIO flModuleIO,
                ModuleIO frModuleIO,
                ModuleIO blModuleIO,
                ModuleIO brModuleIO,
                Field2d field,
                Consumer<Pose2d> resetSimulationPoseCallBack
                ) {
                this.field = field;
            this.gyroIO = gyroIO;
            this.resetSimulationPose = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);

        AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                new Translation2d(),
                fieldRelative,
                true);
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
          new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
          new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
      }

      /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
      }
    

    /** Returns the current turn angle of the module. */
public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

/** Returns the module states (turn angles and drive velocities) for all of the modules. */
@AutoLogOutput(key = "SwerveStates/Measured")
private SwerveModuleState[] getModuleStates() {
  SwerveModuleState[] states = new SwerveModuleState[4];
  for (int i = 0; i < 4; i++) {
    states[i] = modules[i].getState();
  }
  return states;
}

    /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

    @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
    }

    @Override
    public Command getAuto(String autoName) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAuto'");
    }

    @Override
    public void stopWithX() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopWithX'");
    }

    @Override
    public double getMaxLinearSpeedMetersPerSec() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMaxLinearSpeedMetersPerSec'");
    }

    @Override
    public double getMaxAngularSpeedRadPerSec() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMaxAngularSpeedRadPerSec'");
    }

    @Override
    public Rotation2d getRotation() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation'");
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runVelocity'");
    }

    @Override
    public Pose2d getSimulatedDriveTrainPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSimulatedDriveTrainPose'");
    }

    @Override
    public void setSimulationWorldPose(Pose2d pose2d) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSimulationWorldPose'");
    }
}
