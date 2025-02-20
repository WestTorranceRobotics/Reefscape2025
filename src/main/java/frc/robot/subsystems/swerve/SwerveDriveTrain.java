// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.Constants.SwerveAutoConstants.PathPlannerConstants;

public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new SwerveDriveTrain. */
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final Pigeon2 gyro;
  // private final SwerveDriveOdometry odometer;
  private boolean isTest = false;
  private final SwerveDrivePoseEstimator poseEstimator;
  // private DRIVE_MODE driveMode = DRIVE_MODE.FIELD_ORIENTED;
  private int counter = 0;
  private int visionFrequency = 1;

  private Field2d field;

  private double maxTangentialVelocity = 3;
  private double maxAngleVelocity = 2 * Math.PI;

  // StructTopic<Pose2d> publisher =
  // NetworkTableInstance.getDefault().getStructTopic("SwervePose",
  // Pose2d.struct);

  private SwerveModuleState[] desiredStates = new SwerveModuleState[] {new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
  private SwerveModuleState[] actualStates = new SwerveModuleState[] {new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
  private final StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> actualStatePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("ActualStates", SwerveModuleState.struct).publish();

  private final StructPublisher<ChassisSpeeds> chassisSpeedPublisher = NetworkTableInstance
      .getDefault().getStructTopic("ChassisSpeed", ChassisSpeeds.struct).publish();

  private final DoublePublisher rotationPublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("Rotation").publish();

  // public enum SwerveModuleType {
  // MAG_ENCODER,
  // CANCODER
  // }

  // public enum DRIVE_MODE {
  // FIELD_ORIENTED,
  // ROBOT_ORIENTED,
  // AUTONOMOUS
  // }

  /**
   * Construct a new {@link SwerveDrivetrain}
   */
  public SwerveDriveTrain(final Pigeon2 gyro) {
    // Initializing the modules
    frontLeft = new SwerveModule(SwerveDriveConstants.kFLDriveID, SwerveDriveConstants.kFLTurningID,
        SwerveDriveConstants.kFLDriveReversed, SwerveDriveConstants.kFLTurningReversed,
        CANCoderConstants.kFLCANCoderID, CANCoderConstants.kFLCANCoderReversed,
        CANCoderConstants.kFLEncoderOffset);
    frontRight = new SwerveModule(SwerveDriveConstants.kFRDriveID,
        SwerveDriveConstants.kFRTurningID, SwerveDriveConstants.kFRDriveReversed,
        SwerveDriveConstants.kFRTurningReversed, CANCoderConstants.kFRCANCoderID,
        CANCoderConstants.kFRCANCoderReversed, CANCoderConstants.kFREncoderOffset);
    backLeft = new SwerveModule(SwerveDriveConstants.kBLDriveID, SwerveDriveConstants.kBLTurningID,
        SwerveDriveConstants.kBLDriveReversed, SwerveDriveConstants.kBLTurningReversed,
        CANCoderConstants.kBLCANCoderID, CANCoderConstants.kBLCANCoderReversed,
        CANCoderConstants.kBLEncoderOffset);
    backRight = new SwerveModule(SwerveDriveConstants.kBRDriveID, SwerveDriveConstants.kBRTurningID,
        SwerveDriveConstants.kBRDriveReversed, SwerveDriveConstants.kBRTurningReversed,
        CANCoderConstants.kBRCANCoderID, CANCoderConstants.kBRCANCoderReversed,
        CANCoderConstants.kBREncoderOffset);

    this.gyro = gyro;
    this.gyro.setYaw(0);

    this.poseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics,
        gyro.getRotation2d(), getModulePositions(), new Pose2d());
    // this.odometer = new SwerveDriveOdometry(
    // kDriveKinematics,
    // new Rotation2d(0),
    // getModulePositions());

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }

    field = new Field2d();
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    AutoBuilder.configure(this::getPose, this::resetOdometry, this::getChassisSpeeds,
        this::setChassisSpeeds,
        new PPHolonomicDriveController(PathPlannerConstants.kPPTranslationPIDConstants,
            PathPlannerConstants.kPPRotationPIDConstants),
        config, () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this);
  }

  @Override
  public final void periodic() {
    runModules();
    poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    desiredStates[0] = frontLeft.getDesiredState();
    desiredStates[1] = frontRight.getDesiredState();
    desiredStates[2] = backLeft.getDesiredState();
    desiredStates[3] = backRight.getDesiredState();
    desiredStatePublisher.set(desiredStates);

    actualStates[0] = frontLeft.getState();
    actualStates[1] = frontRight.getState();
    actualStates[2] = backLeft.getState();
    actualStates[3] = backRight.getState();
    actualStatePublisher.set(actualStates);

    rotationPublisher.set(gyro.getRotation2d().getRadians());
    chassisSpeedPublisher.set(getChassisSpeeds());

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {

  }

  // ****************************** RESETTERS ******************************/

  /**
   * Resets the odometry to given pose
   * 
   * @param pose A Pose2D representing the pose of the robot
   */
  public void resetOdometry(final Pose2d pose) {
    // odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public final void refreshModulePID(final double kPDrive, final double kIDrive,
      final double kDDrive, final double kVDrive, final double kPTurning, final double kITurning,
      final double kDTurning) {
    frontLeft.refreshPID(kPDrive, kIDrive, kDDrive, kVDrive, kPTurning, kITurning, kDTurning);
    backLeft.refreshPID(kPDrive, kIDrive, kDDrive, kVDrive, kPTurning, kITurning, kDTurning);
    frontRight.refreshPID(kPDrive, kIDrive, kDDrive, kVDrive, kPTurning, kITurning, kDTurning);
    backRight.refreshPID(kPDrive, kIDrive, kDDrive, kVDrive, kPTurning, kITurning, kDTurning);
  }

  /**
   * Stops all modules. See {@link SwerveModule#stop()} for more info.
   */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * Have modules move to their desired states. See {@link SwerveModule#run()} for more info.
   */
  public void runModules() {
    frontLeft.run();
    frontRight.run();
    backLeft.run();
    backRight.run();
  }

  public void simRunModules() {

  }

  // ****************************** GETTERS ******************************/

  public final double getMaxTanVelocity() {
    return maxTangentialVelocity;
  }

  public final double getAng() {
    return maxAngleVelocity;
  }

  /**
   * Gets a pose2d representing the position of the drivetrain
   * 
   * @return A pose2d representing the position of the drivetrain
   */
  public Pose2d getPose() {
    // return odometer.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the position of each swerve module
   * 
   * @return An array of swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
        backLeft.getPosition(), backRight.getPosition()};

  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(),
        backLeft.getState(), backRight.getState()};
  }

  public final ChassisSpeeds getChassisSpeeds() {
    return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public final void drive(final double xSpeed, final double ySpeed, final double turnSpeed) {
    setModuleStates(SwerveDriveConstants.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, turnSpeed)));
  }

  public final void drive(final double xSpeed, final double ySpeed) {
    drive(xSpeed, ySpeed, 0);
  }

  public final void driveFieldOriented(final double xSpeed, final double ySpeed,
      final double turnSpeed) {
    setModuleStates(SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, gyro.getRotation2d())));
  }

  public final void driveFieldOriented(final double xSpeed, final double ySpeed) {
    driveFieldOriented(xSpeed, ySpeed, 0);
  }

  public final void setChassisSpeeds(final ChassisSpeeds speeds) {
    SwerveModuleState[] targetStates =
        SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(targetStates);
  }

  public final Rotation2d getDriveHeading() {
    return gyro.getRotation2d();
  }

  // ****************************** SETTERS ******************************/

  public final void setTan(final double speed) {
    maxTangentialVelocity = speed;
  }

  public final void setAng(final double speed) {
    maxAngleVelocity = speed;
  }

  public final void setVelocityControl(final boolean withVelocityControl) {
    frontLeft.toggleVelocityControl(withVelocityControl);
    frontRight.toggleVelocityControl(withVelocityControl);
    backLeft.toggleVelocityControl(withVelocityControl);
    backRight.toggleVelocityControl(withVelocityControl);
  }

  /**
   * Set the neutral modes of all modules.
   * <p>
   * true sets break mode, false sets coast mode
   * 
   * @param breaking Whether or not the modules should be in break
   */
  public void setBreak(final boolean breaking, final boolean turnbreaking) {
    frontLeft.setBreak(breaking, turnbreaking);
    frontRight.setBreak(breaking, turnbreaking);
    backLeft.setBreak(breaking, turnbreaking);
    backRight.setBreak(breaking, turnbreaking);
  }

  /**
   * Sets module desired states
   * 
   * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
   */
  public void setModuleStates(final SwerveModuleState[] desiredStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
    // SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public final void towModules() {
    frontLeft.setDesiredState(SwerveDriveConstants.towModuleStates[0], false);
    frontRight.setDesiredState(SwerveDriveConstants.towModuleStates[1], false);
    backLeft.setDesiredState(SwerveDriveConstants.towModuleStates[2], false);
    backRight.setDesiredState(SwerveDriveConstants.towModuleStates[3], false);
  }

  /**
   * Reset the odometry to the specified position.
   * 
   * @param pose
   */
  public void setPoseMeters(final Pose2d pose) {
    poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public final void initMainShuffleboard(final int level) {

    // Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal

    if (level == 0) {
      return;
    }
    ShuffleboardTab tab;
    if (level == 3) {
      tab = Shuffleboard.getTab("Main");
    } else {
      tab = Shuffleboard.getTab("Swerve");
    }

    switch (level) {
      case 0:
        break;
      case 1:
        tab.add("Field Position", field).withSize(6, 3);
        tab.addString(("Current Command"), () -> {
          Command currCommand = this.getCurrentCommand();
          if (currCommand == null) {
            return "null";
          } else {
            return currCommand.getName();
          }
        });
        tab.add("Toggle Test", Commands.runOnce(() -> isTest = !isTest));
        tab.addBoolean("Test Mode", () -> isTest);
        // Might be negative because our swerveDriveKinematics is flipped across the Y
        // axis
      case 2, 3:
        tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
        tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
        tab.addNumber("Odometry Angle",
            () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        tab.addNumber("Driver Heading", () -> getDriveHeading().getDegrees());

        break;
      default:
        break;
    }
  }

  public final void initModuleShuffleboard(final int level) {
    frontRight.initShuffleboard(level);
    frontLeft.initShuffleboard(level);
    backLeft.initShuffleboard(level);
    backRight.initShuffleboard(level);
  }

}
