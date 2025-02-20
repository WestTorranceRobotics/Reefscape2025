// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SubSystemConfigs {

    public static final boolean kEnableWrist = true;
    public static final boolean kEnableShooter = false;
    public static final boolean kEnableIntake = false;
    public static final boolean kEnableIndexer = false;
    public static final boolean kEnableArm = true;

    // Through Bore Encoder Configs
    public static final double kEncoderOffset = 0;
    public static final double kEncoderPositionToAngle = 0;

    // Preset Position
    public static final double kStowPosition = 0;

  }

  public static class DriveConstants {
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;
    public static final double kErrorBound = 0;

    public static final double kTanDeadband = 0.15;
    public static final double kAngDeadband = 0.15;

  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    // public static final double kDriveDistanceLoss = 0.95; // from measuring IRL
    public static final double kDriveDistanceLoss = 1; // from measuring IRL
    public static final double kMetersPerRevolution =
        kWheelDiameterMeters * Math.PI * kDriveDistanceLoss;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution;
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec =
        kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec =
        kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.001;

    public static final double kPTurning = 0.0075; // 0.6
    public static final double kITurning = 0;
    public static final double kDTurning = 0;
    public static final double kFTurning = 0;

    // public static final double kPDrive = 0.00016; // 0.6
    // public static final double kIDrive = 0.1;
    // public static final double kDDrive = 0.01;
    // public static final double kVDrive = 0.11;

    public static final double kPDrive = 0; // 0.6
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;
    public static final double kVDrive = 0.11;

    public static final String kCANivoreName = "rio";

  }

  public static final class SwerveDriveConstants {

    // Distance between
    private double tanDeadband = 0.15;
    private double angDeadband = 0.15;
    public static final double kTrackWidth = 0.635;
    // Distance between front and back wheels
    public static final double kWheelBase = 0.635;

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2));

    public static final int kFRDriveID = 23;
    public static final int kFLDriveID = 24;
    public static final int kBLDriveID = 22;
    public static final int kBRDriveID = 21;

    public static final int kFRTurningID = 13;
    public static final int kFLTurningID = 14;
    public static final int kBLTurningID = 12;
    public static final int kBRTurningID = 11;

    public static final boolean kFRTurningReversed = true;
    public static final boolean kFLTurningReversed = true;
    public static final boolean kBLTurningReversed = true;
    public static final boolean kBRTurningReversed = true;

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;
    public static final boolean kBLDriveReversed = false;
    public static final boolean kBRDriveReversed = false;

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 33;
      public static final int kFLCANCoderID = 34;
      public static final int kBLCANCoderID = 32;
      public static final int kBRCANCoderID = 31;

      public static final boolean kFRCANCoderReversed = true;
      public static final boolean kFLCANCoderReversed = true;
      public static final boolean kBLCANCoderReversed = true;
      public static final boolean kBRCANCoderReversed = true;

      // public static final double kFLEncoderOffset = 45 + 180; //119.6;
      // public static final double kBLEncoderOffset = -45; //-130.7;
      // public static final double kFREncoderOffset = 45; //33.1;
      // public static final double kBREncoderOffset = 35; //52.82;

      public static final double kFLEncoderOffset = 235.723;
      public static final double kBLEncoderOffset = 332.227;
      public static final double kFREncoderOffset = 35.068;
      public static final double kBREncoderOffset = 43.242;

      // public static final double kFLEncoderOffset = 0;
      // public static final double kBLEncoderOffset = 0;
      // public static final double kFREncoderOffset = 0;
      // public static final double kBREncoderOffset = 0;
    }

    public static final double kPhysicalMaxSpeedMetersPerSecond = 10;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    // TODO: Change deceleration with driver feedback, only in small increments
    // (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -3; // Russell says he likes 2.5 from sims,
                                                          // but keep at 3 until
                                                          // tested on real robot

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond =
        kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons

    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates =
        new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315))};

    public static final double kGravityMPS = 9.80665;
  }

  public static final class SwerveAutoConstants {
    public static final double kPTurnToAngle = SmartDashboard.getNumber("kP Theta Teleop", 6);
    public static final double kITurnToAngle = SmartDashboard.getNumber("kI Theta Teleop", 0);
    public static final double kDTurnToAngle = SmartDashboard.getNumber("kD Theta Teleop", 0.2);
    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;

    public static final class PathPlannerConstants {
      public static final double kPPMaxVelocity = 3;
      public static final double kPPMaxAcceleration = 3;
      public static final double kPPMaxAngularVelocity = Math.PI * 2;
      public static final double kPPMaxAngularAcceleration = Math.PI * 2;
      public static final PathConstraints kPPPathConstraints = new PathConstraints(kPPMaxVelocity,
          kPPMaxAcceleration, kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

      public static final double kPP_P = 0;
      public static final double kPP_I = 0;
      public static final double kPP_D = 0;
      public static final PIDConstants kPPTranslationPIDConstants =
          new PIDConstants(kPP_P, kPP_I, kPP_D);

      public static final double kPP_ThetaP = 0.25;
      public static final double kPP_ThetaI = 0;
      public static final double kPP_ThetaD = 0;
      public static final PIDConstants kPPRotationPIDConstants =
          new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

      public static final boolean kUseAllianceColor = true;
    }
  }

  public static final int kWristMotorID = 31;
  public static final int kWristThroughBoneEncoderAID = 7;
  public static final int kWristThroughBoneEncoderBID = 6;

  public static final double kAllowedError = 3;

  // Motor Config
  public static final double kRotorToSensorRatio = 1;
  public static final double kSensorToMechanismRatio = 1;
  public static final double kDutyCycleNeutralDeadband = 0.01;
  public static final boolean kInvertClockwise = true;
  public static final boolean kIdleBrake = true;

  // Magic Motion Constants
  // double kP, double kI, double kD, double kV, double kS, double kA, double kG,
  // double cruiseVel, double accel
  public static final double kP = 0.75;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kV = 0;
  public static final double kS = 0;
  public static final double kA = 0;
  public static final double kG = 0;
  public static final double kCruiseVel = 0;
  public static final double kAccel = 0;

  // Through Bore Encoder Configs
  public static final double kEncoderOffset = 0;
  public static final double kEncoderPositionToAngle = 0;

  // Preset Position
  public static final double kNeutralPosition = 0;

  public static final double kGroundIntakePosition = -3;
  public static final double kShooterFeedingPosition = 11;
  public static final double kAmpScoringPosition = -3;

}
