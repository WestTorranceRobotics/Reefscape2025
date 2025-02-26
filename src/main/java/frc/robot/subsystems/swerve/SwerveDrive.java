package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveDrive extends Subsystem {
  void drive(Translation2d translation, double rotation, boolean fieldCentric, boolean isOpenLoop);

  void setPose(Pose2d pose);

  Pose2d getPose();

  SwerveModuleState[] getModuleStates();

  void setModuleStates(SwerveModuleState[] swerveModuleStates);

  default Rotation2d getHeading() {
    return getPose().getRotation();
  }

  default void setHeading(Rotation2d rotation) {
    setPose(new Pose2d(getPose().getTranslation(), rotation));
  }

  default void zeroHeading() {
    setHeading(Rotation2d.kZero);
  }

  default void initSwerveShuffleboard() {
    SmartDashboard.putData("SwerveDrive", (builder) -> {
      builder.setSmartDashboardType("SwerveDrive");

      builder.addDoubleProperty("Front Left Angle", getModuleStates()[0].angle::getRadians, null);
      builder.addDoubleProperty("Front Left Velocity",
          () -> getModuleStates()[0].speedMetersPerSecond, null);

      builder.addDoubleProperty("Front Right Angle", getModuleStates()[1].angle::getRadians, null);
      builder.addDoubleProperty("Front Right Velocity",
          () -> getModuleStates()[1].speedMetersPerSecond, null);

      builder.addDoubleProperty("Back Left Angle", getModuleStates()[2].angle::getRadians, null);
      builder.addDoubleProperty("Back Left Velocity",
          () -> getModuleStates()[2].speedMetersPerSecond, null);

      builder.addDoubleProperty("Back Right Angle", getModuleStates()[3].angle::getRadians, null);
      builder.addDoubleProperty("Back Right Velocity",
          () -> getModuleStates()[3].speedMetersPerSecond, null);

      builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
    });

    SmartDashboard.putNumber("SwerveModules", getModuleStates()[0].angle.getDegrees());
  }
}
