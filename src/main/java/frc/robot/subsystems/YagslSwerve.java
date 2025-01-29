package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * SwerveSubsystem created by loading in YAGSL.
 */
public class YagslSwerve extends SubsystemBase {

  // private static YagslSwerve instance;
  private SwerveDrive swerve;

  // private final StructPublisher<Pose2d> posePublisher =
  // NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  // private final ShuffleboardTab mapleSimTab = Shuffleboard.getTab("mapleSim");


  // public static YagslSwerve getInstance() {
  // if (instance == null) {
  // instance = new YagslSwerve(FilePath);
  // }
  // return instance;
  // }

  /**
   * Creates a new SwerveSubsystem.
   * 
   * @param directory Folder containing config jsons.
   */
  public YagslSwerve(File directory) {
    try {
      swerve = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED);

      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      swerve.setHeadingCorrection(false);
      swerve.setCosineCompensator(false);
      // swerve.setAutoCenteringModules(true);

      configurePathPlanner();

      // this.leftBackEncoderTab = Shuffleboard.getTab("leftBackEncoder");
      // this.rightBackEncoderTab = Shuffleboard.getTab("rightBackEncoder");
      // this.rightFrontEncoderTab = Shuffleboard.getTab("rightFrontEncoder");
      // this.leftFrontEncoderTab = Shuffleboard.getTab("leftFrontEncoder");

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * PathPlanner setup from
   * https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib.
   */
  private void configurePathPlanner() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotVelocity,
          (speeds, feedforwards) -> drive(speeds),
          new PPHolonomicDriveController(
              PathPlannerConstants.kPPRotationPIDConstants,
              PathPlannerConstants.kPPTranslationPIDConstants),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public SwerveDrive getSwerve() {
    return swerve;
  }

  /**
   * Resets swerve odometry to the gien pose.
   * 
   * @param pose A Pose2d object.
   */
  public void resetPose(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot.
   * 
   * @return A {@link ChassisSpeeds} object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerve.getRobotVelocity();
  }

  /**
   * Gets the measured field-relative robot velocity (x, y and omega).
   * 
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerve.getFieldVelocity();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the MapleSim simulated drive if in simulation, otherwise throws an error.
   * 
   * @return The {@link SwerveDriveSimulation} simulated drive.
   */
  public SwerveDriveSimulation getMapleSimDrive() {
    return swerve.getMapleSimDrive().orElseThrow();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks: one for speeds in which
   * direction, the other for the angle of the robot. Max speed is from
   * {@link SwerveConstants#MAX_SPEED}.
   * 
   * @param xInput Joystick input for the robot to move in the X direction.
   * @param yInput Joystick input for the robot to move in the Y direction.
   * @param headingX Joystick input which controls the angle of the robot.
   * @param headingY Joystick input which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX,
      double headingY) {
    return swerve.swerveController.getTargetSpeeds(
        xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        SwerveConstants.MAX_SPEED);
  }

  /**
   * Primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate
   * ({@link ChassisSpeeds#omegaRadiansPerSecond}, for example), and calculates and commands module
   * states accordingly. Defaults to closed-loop control.
   * 
   * @param translation A {@link Translation2d} that is the commanded linear velocity of the robot,
   *        in meters per second.
   *        <ul>
   *        <li>In robot-relative mode, <code>positive x</code> is torwards the front (bow) and
   *        <code>positive y</code> is torwards the left (port).</li>
   *        <li>In field-relative mode, <code>positive x</code> is away from the alliance wall
   *        (field North) and <code>positive y</code> is torwards the left wall when looking through
   *        the driver station glass (field West).</li>
   *        </ul>
   * @param rotation Robot angular rate, in radians per second. CounterClockWise is positive.
   *        Unaffected by field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerve.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Secondary method for controlling the drivebase. Given a simple {@link ChassisSpeeds}, set the
   * swerve module states to achieve the goal.
   * 
   * @param velocity Desired robot-oriented {@link ChassisSpeeds} for the robot to achieve.
   */
  public void drive(ChassisSpeeds velocity) {
    swerve.drive(velocity);
  }


  /**
   * Secondary method of controlling the drive base, adjusted for field oriented use.
   * 
   * @param velocity Desired {@link ChassisSpeeds} for the robot to achieve.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerve.driveFieldOriented(velocity);
  }


  /**
   * Secondary method of controlling the drive base, adjusted for field oriented use.
   * 
   * @param velocity Desired {@link ChassisSpeeds} for the robot to achieve.
   */
  public void driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    swerve.driveFieldOriented(velocity.get());
  }

  /**
   * Commands robot to continously run {@link YagslSwerve#driveFieldOriented}.
   * 
   * @param translationX Joystick input for the robot to move in the X direction.
   * @param translationY Joystick input for the robot to move in the Y direction.
   * @param headingX Joystick input which controls the angle of the robot.
   * @param headingY Joystick input which controls the angle of the robot.
   * @return A continous {@link Command} driving the robot
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

      driveFieldOriented(
          swerve.swerveController.getTargetSpeeds(
              scaledInputs.getX(),
              scaledInputs.getY(),
              headingX.getAsDouble(),
              headingY.getAsDouble(),
              getHeading().getRadians(),
              SwerveConstants.MAX_SPEED));
    });
  }

  /**
   * Stops all modules.
   */
  public void stop() {
    swerve.drive(new ChassisSpeeds(0, 0, 0));
  }

  /**
   * Uses Limelight MegaTag2 to update odometry pose.
   * 
   * <p>
   * From
   * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2.
   * </p>
   */
  public void updateOdometry() {
    boolean useMegaTag2 = true;

    if (useMegaTag2) {
      // Reset robot orientation
      LimelightHelpers.SetRobotOrientation("",
          swerve.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0,
          0, 0, 0, 0);

      var mt2Estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

      // If there's at least 1 AprilTag in view, update pose
      if (mt2Estimate.tagCount > 0) {
        double xStdDev = 0.7;
        double yStdDev = 0.7;
        double thetaStdDev = 9999999;

        // The lower the standard deviation (StdDev), the more we trust the camera's estimate.
        // We trust x and y a fair amount, and theta (the angle) not at all lmao
        // These values come default from
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2.
        swerve.swerveDrivePoseEstimator
            .setVisionMeasurementStdDevs(VecBuilder.fill(xStdDev, yStdDev, thetaStdDev));

        swerve.addVisionMeasurement(
            mt2Estimate.pose,
            mt2Estimate.timestampSeconds);
      }
    } else {
      var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
      boolean rejectUpdate = false;

      if (mt1Estimate.tagCount == 0) {
        rejectUpdate = true;
      }
      if (mt1Estimate.tagCount == 1 && mt1Estimate.rawFiducials.length == 1) {
        if (mt1Estimate.rawFiducials[0].ambiguity > .7) {
          rejectUpdate = true;
        }
        if (mt1Estimate.rawFiducials[0].distToCamera > 3) {
          rejectUpdate = true;
        }
      }

      if (!rejectUpdate) {
        swerve.addVisionMeasurement(mt1Estimate.pose, mt1Estimate.timestampSeconds);
      }

    }
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      // updateOdometry();
    }

  }

  @Override
  public void simulationPeriodic() {
    // posePublisher.set(getPose());
  }
}
