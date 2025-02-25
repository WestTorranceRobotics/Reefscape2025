// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveDriveJoystickCommand;
import frc.robot.commands.swerve.SwerveJoystickCommand;
import frc.robot.subsystems.swerve.MapleSimSwerveDrive;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandPS4Controller driveController = new CommandPS4Controller(1);
  CommandJoystick joystick = new CommandJoystick(0);

  private Pigeon2 gyro = new Pigeon2(9);
  private SwerveDriveTrain swerveSubsystem;
  private SwerveDrive swerveDrive;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (Robot.isReal()) {
      this.swerveSubsystem = new SwerveDriveTrain(gyro);
      swerveSubsystem.setDefaultCommand(
          new SwerveJoystickCommand(driveController::getLeftY, driveController::getLeftX,
              driveController::getRightX, swerveSubsystem));
      initShuffleboard();
    } else {
      this.swerveDrive = new MapleSimSwerveDrive();
      this.swerveDrive.setDefaultCommand(
          new SwerveDriveJoystickCommand(joystick::getY, joystick::getX, driveController::getRightX,
              swerveDrive));
    }
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {

    driveController.R2().onTrue(Commands.runOnce(() -> {
      gyro.setYaw(0);
      driveController.setRumble(RumbleType.kBothRumble, 0);

    }));

    // driveAbutton.whileTrue(new ArmPercentCommand(arm, 0.75, false));
    // driveBbutton.whileTrue(new ArmPercentCommand(arm, -0.75, false));
    // driveAbutton.whileTrue(new IndexerPercentCommand(indexer, -0.2));
    // driveBbutton.whileTrue(new IndexerPercentCommand(indexer, 0.2));

    // driveXbutton.whileTrue(new WristPercentCommand(wrist, 0.1, false));
    // driveYbutton.whileTrue(new WristPercentCommand(wrist, -0.1, false));

    // driveLeftTriggerButton.whileTrue(new IntakePercentCommand(intake, 0.2));
    // driveRightTriggerButton.whileTrue(new IntakePercentCommand(intake, -0.2));

    // // driveAbutton.whileTrue(new IntakePercentCommand(intake, 0.5));
    // // driveBbutton.whileTrue(new IntakePercentCommand(intake, -0.5));

    // driveLeftBumperButton.whileTrue(new shooterPercentCommand(shooter, 1.0));
    // driveRightBumperButton.whileTrue(new shooterPercentCommand(shooter, -1.0));

    // // driverUpPOVButton.whileTrue(new ArmTargetPositionManual(arm, 31));
    // // driverDownPOVButton.whileTrue(new WristTargetPositionManual(wrist, -3));

    // driverDownPOVButton.whileTrue(new NeutralPositionCommand(arm, wrist,
    // intake));
    // driverRightPOVButton.whileTrue(new GroundIntakeCommand(arm, intake, wrist));
    // driverLeftPOVButton.onTrue(new ShooterFeedingCommand(arm, intake, wrist,
    // shooter));
    // driverUpPOVButton.onTrue(new AmpScoringCommand(arm, wrist, intake));

    // // driverLeftPOVButton.onTrue(new intakeControlledCommand(intake, 2400));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

  }

  public final void initShuffleboard() {
    swerveSubsystem.initModuleShuffleboard(1);
    swerveSubsystem.initMainShuffleboard(1);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");

    // An example command will be run in autonomous
    // return null;
  }
}
