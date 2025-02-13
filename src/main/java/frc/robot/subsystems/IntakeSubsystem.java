package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command; 
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;

// NOTE: Currently a work-in-progress.

public class IntakeSubsystem extends SubsystemBase {
  
  /* creates a motor object for the Intake's hinge
  *  - Right now, I'm not sure if the device ID is correct.
  *  - All original code can be found within the ElevatorSubsystem file, which has a link to the manual.
  */
  final TalonFX m_IntakeHinge = new TalonFX(0);

  public IntakeSubsystem() {

    var slot0Configs = new Slot0Configs();
    // kP value for intake; current value is a placeholder
    slot0Configs.kP = 0.1;
    // kG value for intake (to account for gravity); current value is a placeholder
    slot0Configs.kG = 0.01;

    m_IntakeHinge.getConfigurator().apply(slot0Configs);
  

  }

   /*  This autonomous command rotates the intake with a singular TalonFX motor to reach the designated angle
   *    -current values are placeholders
   */ 
  public Command autonomousRotateIntake() {
  
    return runOnce(() -> {
      
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
      // commands intake hinge motor to run until desired angle is reached
      m_IntakeHinge.setControl(m_request.withVelocity(2).withFeedForward(0.1));


    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

}

}
