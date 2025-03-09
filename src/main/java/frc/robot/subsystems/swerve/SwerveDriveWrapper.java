package frc.robot.subsystems.swerve;

import com.sun.tools.jconsole.JConsoleContext;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import jdk.jshell.spi.ExecutionControl;

public class SwerveDriveWrapper implements SwerveDrive {
  private final SwerveDriveTrain swerveDriveTrain;

  public SwerveDriveWrapper(SwerveDriveTrain swerveDriveTrain) {
    this.swerveDriveTrain = swerveDriveTrain;
  }

  @Override
  public void drive(Translation2d translation, double rotation, boolean _fieldCentric,
      boolean _isOpenLoop) {
    this.swerveDriveTrain.drive(translation.getX(), translation.getY(), rotation);
  }

  @Override
  public void setPose(Pose2d pose) {
    this.swerveDriveTrain.setPoseMeters(pose);
  }

  @Override
  public Pose2d getPose() {
    return this.swerveDriveTrain.getPose();
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    return null;
  }

  @Override
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    this.swerveDriveTrain.setModuleStates(swerveModuleStates);
  }

  @Override
  public void initSwerveShuffleboard() {

  }
}
