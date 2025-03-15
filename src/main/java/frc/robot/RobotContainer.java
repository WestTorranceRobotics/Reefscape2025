// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.swerve.SwerveJoystickCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  CommandPS4Controller driveController = new CommandPS4Controller(0);

  private SwerveDriveTrain swerveSubsystem;
  private Intake intake;
  private Pigeon2 gyro;

  private Command runIntakeAuto;
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Pigeon2 gyro) {
    this.swerveSubsystem = new SwerveDriveTrain(gyro);
    this.intake = new Intake(true);
    this.gyro = gyro;

    this.runIntakeAuto = Commands.sequence(
        intake.c_directSetIntakeSpeedCommand(0.2),
        Commands.waitSeconds(1),
        intake.c_stopCommand());

    NamedCommands.registerCommand("RunIntakeAuto", runIntakeAuto);

    swerveSubsystem.setDefaultCommand(
        new SwerveJoystickCommand(
            driveController::getLeftY,
            driveController::getLeftX,
            driveController::getRightX,
            swerveSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser("Main auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    initShuffleboard();
  }

  private void configureBindings() {
    // circle -- Outtake fast
    driveController.circle().onTrue(intake.c_directSetIntakeSpeedCommand(0.185));
    driveController.circle().onFalse(intake.c_stopCommand());

    // cross -- Outtake slow
    driveController.cross().onTrue(intake.c_directSetIntakeSpeedCommand(0.16));
    driveController.cross().onFalse(intake.c_stopCommand());

    // Reset gyro
    driveController.L1().onTrue(Commands.runOnce(() -> {
      gyro.setYaw(0);
    }));

    // Run intake backwards to unwedge coral
    driveController.R1().onTrue(intake.c_directSetIntakeSpeedCommand(-0.5));
    driveController.R1().onFalse(intake.c_stopCommand());
  }

  /**
   * Set up Shuffleboard logs here.
   */
  public final void initShuffleboard() {
    swerveSubsystem.initModuleShuffleboard(1);
    swerveSubsystem.initMainShuffleboard(1);
    intake.initShuffleboard();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
