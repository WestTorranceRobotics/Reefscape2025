package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  // m_leftElevator is the leading motor
  final TalonFX m_leftElevator = new TalonFX(0);
  // m_rightElevator is the following motor
  final TalonFX m_rightElevator = new TalonFX(1);

  

  public ElevatorSubsystem() {

    var slot0Configs = new Slot0Configs();
    // kP value for elevator PID; current value is a placeholder
    slot0Configs.kP = 1;
    // kG value for elevator PID (to account for gravity); current value is a placeholder
    slot0Configs.kG = 0.1;


    m_leftElevator.getConfigurator().apply(slot0Configs);
    m_rightElevator.getConfigurator().apply(slot0Configs);

    m_rightElevator.setControl(new Follower(m_leftElevator.getDeviceID(), false));
    
    m_leftElevator.setInverted(false);
    m_rightElevator.setInverted(true);


  }

  /* Command function that instructs the elevator to reach level three on the coral reef.
   * Currently, I'm not exactly sure where to paste the code from the CTR Instructions Manual (https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/index.html),
   * so as of right now I've put all of it into this ElevatorSubsystem class.
  */

  public Command autonomousReachL3Command() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
  /* 
   *  The following two lines command the left and right elevator motors, respectively, to run until level THREE on the coral reef is reached in AUTONOMOUS mode.
   *  Current values are placeholders.
   *  -Code was copied and pasted from the CTR Electronics Manual (https://v6.docs.ctr-electronics.com/en/stable/index.html)
   *    under Device API > TalonFX > Basic PID and Profiling 
  */ 
      m_leftElevator.setControl(m_request.withVelocity(6).withFeedForward(0.5));
      /* one-time action goes here */
    });
  }

  public Command reachL1Command() {
    
    return runOnce(() -> {
      
      // not sure if this line is redundant or not
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
  /* 
   *  The following two lines command the left and right elevator motors, respectively, to run until level ONE on the coral reef is reached in TELEOP.
   *  -Current values are placeholders.
   *  -newFeedForward should increase depending on the height needed to reach
   *   because motors fight gravity to a greater extent as the target height increases.
  */ 
      m_leftElevator.setControl(m_request.withVelocity(2).withFeedForward(0.3));

    });
  }

  public Command reachL2Command() {
    
    return runOnce(() -> {
      
      // not sure if this line is redundant or not
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
  /* 
   *  The following two lines command the left and right elevator motors, respectively, to run until level TWO on the coral reef is reached in TELEOP.
   *  Current values are placeholders.
  */ 
      m_leftElevator.setControl(m_request.withVelocity(4).withFeedForward(0.4));

    });
  }

  public Command reachL3Command() {
    
    return runOnce(() -> {
      
      // not sure if this line is redundant or not
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
  /* 
   *  The following two lines command the left and right elevator motors, respectively, to run until level THREE on the coral reef is reached in TELEOP.
   *  Current values are placeholders.
  */ 
      m_leftElevator.setControl(m_request.withVelocity(6).withFeedForward(0.5));

    });
  }

  public Command reachL4Command() {
    
    return runOnce(() -> {
      
      // not sure if this line is redundant or not
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
      
  /* 
   *  The following two lines command the left and right elevator motors, respectively, to run until level FOUR on the coral reef is reached in TELEOP.
   *  Current values are placeholders.
  */ 
      m_leftElevator.setControl(m_request.withVelocity(8).withFeedForward(0.6));

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
