package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDcmd extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem mSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorPIDcmd(ElevatorSubsystem subsystem) {
    this.mSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    
    m_leftElevator.setControl(m_request.withVelocity(8).withFeedForward(0.5));
    m_rightElevator.setControl(m_request.withVelocity(8).withFeedForward(0.5));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;


}
}