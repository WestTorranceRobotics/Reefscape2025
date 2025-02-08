// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.YagslSwerve;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */

/**
 * Uses {@link SwerveInputStream} and {@link YagslSwerve#driveFieldOriented()} to drive.
 */
public class RelativeAngleDrive extends Command {
  private final YagslSwerve swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier headingHorizontal;

  /**
   * Creates a new RelativeDrive.
   */
  public RelativeAngleDrive(YagslSwerve swerve, DoubleSupplier vX, DoubleSupplier vY,
      DoubleSupplier headingHorizontal) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerve(),
        () -> vY.getAsDouble(),
        () -> vX.getAsDouble())
        .withControllerRotationAxis(() -> -headingHorizontal.getAsDouble())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.3)
        .allianceRelativeControl(true);

    swerve.driveFieldOriented(driveAngularVelocity);
    // swerve.drive(driveAngularVelocity.get());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
