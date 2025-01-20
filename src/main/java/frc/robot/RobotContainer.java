// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DirectAngleDrive;
import frc.robot.subsystems.YagslSwerve;
import org.ironmaple.simulation.SimulatedArena;


/**
 * Container for the entire robot.
 */
public class RobotContainer {
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // SUBSYSTEMS
  private final YagslSwerve swerve = new YagslSwerve(SwerveConstants.SWERVE_FILE_PATH);

  // COMMANDS
  private final DirectAngleDrive directAngleDrive =
      new DirectAngleDrive(
          swerve,
          () -> -driverController.getLeftX(),
          () -> -driverController.getLeftY(),
          () -> -driverController.getRightX(),
          () -> -driverController.getRightY());

  /**
   * Creates a new Robot.
   */
  public RobotContainer() {
    // if (Robot.isSimulation()) {
    // // System.out.println(">>>> Creating swerve sim for the " + counter + "th time");
    // // counter++;
    // // SimulatedArena.getInstance()
    // // .addDriveTrainSimulation(swerve.getSimulation());
    // }

    configureBindings();
  }


  private void configureBindings() {
    swerve.setDefaultCommand(directAngleDrive);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
    // return new WaitCommand(2);
  }

}
