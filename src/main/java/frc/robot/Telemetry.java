package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {
  private static Telemetry instance = null;

  public static Telemetry getInstance() {
    if (instance == null)
      instance = new Telemetry();
    return instance;
  }

  private final Field2d field;

  public Telemetry() {
    this.field = new Field2d();
    SmartDashboard.putData("Field", field);
  }


  public Field2d getField() {
    return field;
  }

  public void setSimulationPose(Pose2d realPose) {
    field.setRobotPose(realPose);
  }
}
