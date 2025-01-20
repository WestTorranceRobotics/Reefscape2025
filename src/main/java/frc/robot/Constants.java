// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.YagslSwerve;
import java.io.File;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * TODO: Add documentation.
   */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * {@link YagslSwerve} constants.
   */
  public static class SwerveConstants {
    public static final double MAX_SPEED = 10;
    public static final File SWERVE_FILE_PATH = new File(Filesystem.getDeployDirectory(), "swerve");

    /**
     * Motor, encoder, and gyro ids.
     */
    public static class SwerveIds {
      // TalonFX drive motor controllers
      public static final int leftFrontDriveId = 1;
      public static final int leftBackDriveId = 2;
      public static final int rightBackDriveId = 3;
      public static final int rightFrontDriveId = 4;

      // CANCoders (encoders) for the TalonFX drive motors
      public static final int leftBackCanCoderId = 5;
      public static final int leftFrontCanCoderId = 6;
      public static final int rightFrontCanCoderId = 7;
      public static final int rightBackCanCoderId = 8;

      // Pigeon 2 gyro
      public static final int pigeon2GyroId = 9;

      // SwerveMax angle motor controller IDs
      public static final int leftFrontAngleId = 11;
      public static final int leftBackAngleId = 12;
      public static final int rightBackAngleId = 13;
      public static final int rightFrontAngleId = 14;
    }
  }

  /**
   * Pathplanner constants.
   */
  public static final class PathPlannerConstants {
    public static final double kPPMaxVelocity = 3;
    public static final double kPPMaxAcceleration = 3;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;
    public static final PathConstraints kPPPathConstraints =
        new PathConstraints(
            kPPMaxVelocity,
            kPPMaxAcceleration,
            kPPMaxAngularVelocity,
            kPPMaxAngularAcceleration);

    // Translation PID constants
    public static final double kPP_P = 0;
    public static final double kPP_I = 0;
    public static final double kPP_D = 0;
    public static final PIDConstants kPPTranslationPIDConstants =
        new PIDConstants(kPP_P, kPP_I, kPP_D);

    // Rotation PID constants
    public static final double kPP_ThetaP = 0.25;
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0;
    public static final PIDConstants kPPRotationPIDConstants =
        new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;
  }
}
