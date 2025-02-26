// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.util.function.DoubleSupplier;

public class SwerveDriveJoystickCommand extends Command {
  /** Creates a new SwerveJoystickCommand. */
  private final SwerveDrive drive;


  private double modifyInputs(double val, boolean isRot) {
    if (isRot) {
      if (Math.abs(val) < DriveConstants.kAngDeadband) {
        val = 0;
      }
      return val * drive.getHeading().getRadians();
    } else {
      if (Math.abs(val) < DriveConstants.kTanDeadband) {
        val = 0;
      }
      return val * 6;
    }
  }

  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier z;

  public SwerveDriveJoystickCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot,
      SwerveDrive instance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.x = fwd;
    this.y = str;
    this.z = rot;

    drive = instance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(new Translation2d(modifyInputs(-x.getAsDouble(), false),
        modifyInputs(-y.getAsDouble(), false)), modifyInputs(z.getAsDouble(), true), false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(0, 0), 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
