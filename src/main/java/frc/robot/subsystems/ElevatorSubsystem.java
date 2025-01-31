package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  final TalonFX m_leftElevator = new TalonFX(0);
  final TalonFX m_rightElevator = new TalonFX(1);

  public ElevatorSubsystem() {

    var slot0Configs = new Slot0Configs();
    // kP value for elevator PID; current value is a placeholder
    slot0Configs.kP = 1;
    // kG value for elevator PID; current value is a placeholder
    slot0Configs.kG = 0.1;

    m_leftElevator.getConfigurator().apply(slot0Configs);
    m_rightElevator.getConfigurator().apply(slot0Configs);
    
    
  }

  public Command autonomousReachL3() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
  /* 
      The following two lines command the left and right elevator motors, respectively, to run until level three on the coral reef is reached.
      * Code was copied and pasted from the CTR Electronics Manual (https://v6.docs.ctr-electronics.com/en/stable/index.html)
        under Device API > TalonFX > Basic PID and Profiling 
  */ 
      m_leftElevator.setControl(m_request.withVelocity(8).withFeedForward(0.5));
      m_rightElevator.setControl(m_request.withVelocity(8).withFeedForward(0.5));
      /* one-time action goes here */
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
