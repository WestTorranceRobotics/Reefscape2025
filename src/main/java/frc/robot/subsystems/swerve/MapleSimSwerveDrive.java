package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Telemetry;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import java.awt.event.TextEvent;

import static edu.wpi.first.units.Units.*;

public class MapleSimSwerveDrive implements SwerveDrive {
  private final SelfControlledSwerveDriveSimulation simulatedDrive;

  public MapleSimSwerveDrive() {
    final DriveTrainSimulationConfig driveTrainSimulationConfig =
        DriveTrainSimulationConfig.Default().withGyro(COTS.ofPigeon2()).withSwerveModule(
            new SwerveModuleSimulationConfig(DCMotor.getFalcon500(1), DCMotor.getNEO(1), 5.143, 12,
                Volts.of(0.1), Volts.of(0.1), Meters.of(0.048), KilogramSquareMeters.of(0.02),
                1.2));

    this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
        new SwerveDriveSimulation(driveTrainSimulationConfig, new Pose2d(0, 0, new Rotation2d())));

    this.simulatedDrive.withCurrentLimits(Amps.of(60), Amp.of(20));

    SimulatedArena.getInstance()
        .addDriveTrainSimulation(this.simulatedDrive.getDriveTrainSimulation());

    this.initSwerveShuffleboard();

    setPose(new Pose2d(2, 2, new Rotation2d()));
  }

  @Override
  public void drive(Translation2d translation, double rotation, boolean fieldCentric, boolean isOpenLoop) {
    this.simulatedDrive.runChassisSpeeds(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation), new Translation2d(),
        fieldCentric, true);
  }

  @Override
  public void setPose(Pose2d pose) {
    this.simulatedDrive.resetOdometry(pose);
    this.simulatedDrive.setSimulationWorldPose(pose);
  }

  @Override
  public Pose2d getPose() {
    return this.simulatedDrive.getOdometryEstimatedPose();
  }

  public Pose2d getSimPose() {
    return this.simulatedDrive.getActualPoseInSimulationWorld();
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    return this.simulatedDrive.getMeasuredStates();
  }

  @Override
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    this.simulatedDrive.runSwerveStates(swerveModuleStates);
  }

  @Override
  public void periodic() {
    this.simulatedDrive.periodic();
    this.simulatedDrive.resetOdometry(getPose());

    Telemetry telemetryInstance = Telemetry.getInstance();

    telemetryInstance.setSimulationPose(this.getSimPose());
    telemetryInstance.setOdometryPose(this.getPose());
    telemetryInstance.setSwerveModuleStates(this.simulatedDrive.getMeasuredStates());
    telemetryInstance.setSwerveModuleTargets(this.simulatedDrive.getSetPointsOptimized());
    telemetryInstance.setSwerveModulePositions(this.simulatedDrive.getLatestModulePositions());
  }

  public AbstractDriveTrainSimulation getSimDrive() {
    return this.simulatedDrive.getDriveTrainSimulation();
  }
}
