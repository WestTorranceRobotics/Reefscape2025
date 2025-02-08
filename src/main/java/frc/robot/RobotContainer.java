// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DirectAngleDrive;
import frc.robot.commands.RelativeAngleDrive;
import frc.robot.subsystems.YagslSwerve;


/**
 * Container for the entire robot.
 */
public class RobotContainer {
  private final CommandPS4Controller driverController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  // SUBSYSTEMS
  private final YagslSwerve swerve = new YagslSwerve(SwerveConstants.SWERVE_FILE_PATH);

  // COMMANDS
  // private final DirectAngleDrive directAngleDrive =
  // new DirectAngleDrive(
  // swerve,
  // driverController::getLeftX,
  // driverController::getLeftY,
  // driverController::getRightX,
  // driverController::getRightY);

  private final RelativeAngleDrive relativeAngleDrive = new RelativeAngleDrive(
      swerve,
      () -> driverController.getLeftX(),
      () -> driverController.getLeftY(),
      () -> driverController.getRightX());

  // TODO: add keyboard simulation turning with one axis



  /**
   * Creates a new Robot.
   */
  public RobotContainer() {

    swerve.setDefaultCommand(relativeAngleDrive);
    configureBindings();
  }


  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
    // return new WaitCommand(2);
  }

}
