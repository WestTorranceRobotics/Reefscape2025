// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.RunIntakeForward;
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

  private Pigeon2 gyro = new Pigeon2(9);
  private SwerveDriveTrain swerveSubsystem = new SwerveDriveTrain(gyro);
  private Intake intake = new Intake(false);

  private RunIntakeForward runIntakeForwardCommand = new RunIntakeForward(intake);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(driveController::getLeftY,
        driveController::getLeftX, driveController::getRightX, swerveSubsystem));
    // intake.setDefaultCommand(intake.idleBackwardsCommand());



    configureBindings();
    initShuffleboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {

    driveController.R1().onTrue(Commands.runOnce(() -> {
      gyro.setYaw(0);
      driveController.setRumble(RumbleType.kLeftRumble, 1);
      driveController.setRumble(RumbleType.kRightRumble, 1);
    }));

    driveController.R1().onFalse(Commands.runOnce(() -> {
      driveController.setRumble(RumbleType.kLeftRumble, 0);
      driveController.setRumble(RumbleType.kRightRumble, 0);
    }));

    driveController.circle().onTrue(intake.c_setIntakeSpeedCommand(120));
    driveController.circle().onFalse(intake.c_stopCommand());

    // driveController.circle().onFalse(Commands.runOnce(intake::stop));

    // driveAbutton.whileTrue(new ArmPercentCommand(arm, 0.75, false));
    // driveBbutton.whileTrue(new ArmPercentCommand(arm, -0.75, false));
    // driveAbutton.whileTrue(new IndexerPercentCommand(indexer, -0.2));
    // driveBbutton.whileTrue(new IndexerPercentCommand(indexer, 0.2));
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
    return new PathPlannerAuto("New Auto");
  }
}
