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
    public static final double DEADBAND = 0.2;
    /*
     * Elevator motor tick count to reach designated reef level for auto period (current value is a
     * placeholder)
     */
    public static final float elevatorTicks = 2000;
    /*
     * Intake motor tick count to angle coral for auto period (current value is a placeholder)
     */
    public static final float intakeTicks = 1000;
    
    
    public static final int kDriverControllerPort = 0;
    public static final float ReefL1_Cm = 46;
    public static final float ReefL2_Cm = 81;
    public static final float ReefL3_Cm = 121;
    public static final float ReefL3_Cm = 183;

    public static final float AlgaeHeight1_Cm = 81;
    public static final float AlgaeHeight2_Cm = 121;
    
    public static final double DEADBAND = 0.2;
  }

  /**
   * {@link YagslSwerve} constants.
   */
  public static class SwerveConstants {
    public static final double MAX_SPEED = 10;
    public static final File SWERVE_FILE_PATH = new File(Filesystem.getDeployDirectory(), "swerve");

    /**
     * Motor, encoder, and gyro ids. Order goes
     * <ol>
     * <li>Back right</li>
     * <li>Back left</li>
     * <li>Front right</li>
     * <li>Front left</li>
     * </ol>
     */
    public static class SwerveIds {
      // TalonFX drive motor controllers
      public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 21;
      public static final int BACK_LEFT_DRIVE_MOTOR_ID = 22;
      public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 23;
      public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 24;

      // CANCoders (encoders) for the TalonFX drive motors
      public static final int BACK_RIGHT_CANCODER_ID = 11;
      public static final int BACK_LEFT_CANCODER_ID = 12;
      public static final int FRONT_RIGHT_CANCODER_ID = 13;
      public static final int FRONT_LEFT_CANCODER_ID = 14;

      // Pigeon 2 gyro
      public static final int PIGEON_2_GYRO_ID = 9;

      // SparkMAX angle motor controller IDs
      public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 11;
      public static final int BACK_LEFT_ANGLE_MOTOR_ID = 12;
      public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 13;
      public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 14;
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
