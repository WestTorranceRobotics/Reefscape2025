// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import java.util.function.DoubleSupplier;

public class SwerveJoystickWithAbsoluteTurningCommand extends Command {
  private final SwerveDriveTrain swerve;

  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier headingHorizontal;
  private DoubleSupplier headingVertical;

  private PIDController anglePidController;
  private double targetAngle;

  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;

  /** Creates a new SwerveJoystickWithAbsoluteTurningCommand. */
  public SwerveJoystickWithAbsoluteTurningCommand(SwerveDriveTrain swerve, DoubleSupplier fwd,
      DoubleSupplier str, DoubleSupplier headingHorizontal, DoubleSupplier headingVertical) {
    this.swerve = swerve;

    this.x = fwd;
    this.y = str;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;

    this.targetAngle = 0;

    this.anglePidController = new PIDController(kP, kI, kD);
    anglePidController.setSetpoint(targetAngle);
    anglePidController.enableContinuousInput(0, 360);
    anglePidController.setTolerance(1);

    addRequirements(swerve);
  }

  private double getAngleFromHeadings(double horiz, double vert) {
    return Math.atan2(horiz, vert);
  }

  private double modifyInputs(double speed, boolean isRotationValue) {
    if (isRotationValue) {
      if (Math.abs(speed) < DriveConstants.kAngDeadband) {
        speed = 0;
      }
      return speed * swerve.getMaxAngVelocity();
    } else {
      if (Math.abs(speed) < DriveConstants.kTanDeadband) {
        speed = 0;
      }
      return speed * swerve.getMaxTanVelocity();
    }
  }

  private void driveFromChassis(ChassisSpeeds speeds) {
    var states = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics
        .desaturateWheelSpeeds(states, SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    swerve.setModuleStates(states);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    anglePidController.setSetpoint(
        getAngleFromHeadings(
            MathUtil.applyDeadband(headingHorizontal.getAsDouble(), 0.05),
            MathUtil.applyDeadband(headingVertical.getAsDouble(), 0.05)));

    double turningSpeedRadians = 0;

    if (anglePidController.atSetpoint()) {
      turningSpeedRadians =
          modifyInputs(anglePidController.calculate(swerve.getDriveHeading().getRadians()), true);
    }

    driveFromChassis(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            modifyInputs(-x.getAsDouble(), false),
            modifyInputs(-y.getAsDouble(), false),
            turningSpeedRadians,
            Rotation2d.fromDegrees(swerve.getDriveHeading().getDegrees())));
  }

  @Override
  public void end(boolean interrupted) {
    driveFromChassis(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
