package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {
  private static Telemetry instance = null;

  private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

  private final NetworkTable driveStateTable = networkTableInstance.getTable("DriveState");

  private final StructPublisher<Pose2d> odometryPose =
      driveStateTable.getStructTopic("OdometryPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> realPose =
      driveStateTable.getStructTopic("RealPose", Pose2d.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> swerveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> swerveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> swerveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

  private final NetworkTable fieldTable = networkTableInstance.getTable("Pose");
  private final DoubleArrayPublisher fieldPublisher =
      fieldTable.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePublisher = fieldTable.getStringTopic(".type").publish();

  public static Telemetry getInstance() {
    if (instance == null)
      instance = new Telemetry();
    return instance;
  }

  public Telemetry() {
  }

  private void setField2d(Pose2d pose) {
    double[] fieldArray = new double[3];
    fieldArray[0] = pose.getX();
    fieldArray[1] = pose.getY();
    fieldArray[2] = pose.getRotation().getDegrees();

    fieldTypePublisher.set("Field2d");
    fieldPublisher.set(fieldArray);

    SignalLogger.writeDoubleArray("DriveState/Pose", fieldArray);
  }

  public void setSimulationPose(Pose2d realPose) {
    this.realPose.set(realPose);
    this.setField2d(realPose);
  }

  public void setOdometryPose(Pose2d simPose) {
    this.realPose.set(simPose);
  }

  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    this.swerveModuleStates.set(swerveModuleStates);
  }

  public void setSwerveModulePositions(SwerveModulePosition[] swerveModulePositions) {
    this.swerveModulePositions.set(swerveModulePositions);
  }

  public void setSwerveModuleTargets(SwerveModuleState[] swerveModuleTargets) {
    this.swerveModuleTargets.set(swerveModuleTargets);
  }
}
