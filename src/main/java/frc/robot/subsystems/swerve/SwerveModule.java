// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveDriveConstants;

/** Add your docs here. */
public class SwerveModule {
  private final TalonFX driveMotor;
  private final SparkMax turnMotor;
  private final CANcoder canCoder;
  private final TalonFXConfigurator driveConfigurator;
  private final SparkMaxConfig sparkConfig;

  private final DCMotorSim sim_driveMotor;

  private final DutyCycleOut driveRequest;
  private final PIDController turnPIDController;
  private final VelocityVoltage driveVelocityRequest;
  private final NeutralOut brakeRequest;

  private Slot0Configs drivePIDConfigs;

  private final int driveMotorID;
  private final int turnMotorID;
  private final int CANCoderID;

  private final boolean invertTurningEncoder;

  private double currentPercent = 0;
  private double currentTurnPercent = 0;
  private double currentAngle = 0;
  private double desiredAngle = 0;
  private double desiredVelocity = 0;
  private boolean velocityControl = true;
  private double encoderOffset = 0;

  private SwerveModuleState desiredState = null;
  private SwerveModulePosition currPosition = new SwerveModulePosition();
  private SwerveModuleState currState = new SwerveModuleState();
  private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

  /**
   * Construct a new CANCoder Swerve Module.
   * 
   * @param driveMotorId
   * @param turningMotorId
   * @param invertDriveMotor
   * @param invertTurningMotor
   * @param CANCoderId
   * @param CANCoderOffsetDegrees
   * @param CANCoderReversed
   */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor,
      boolean invertTurningMotor, int CANCoderId, boolean CANCoderReversed, double encoderOffset) {
    this.driveMotorID = driveMotorId;
    this.turnMotorID = turningMotorId;
    this.CANCoderID = CANCoderId;
    this.driveMotor = new TalonFX(driveMotorId, ModuleConstants.kCANivoreName);
    this.turnMotor = new SparkMax(turningMotorId, SparkMax.MotorType.kBrushless);

    this.desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    this.sim_driveMotor =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), 0.001,
            ModuleConstants.kDriveMotorGearRatio), DCMotor.getFalcon500Foc(1));

    this.driveConfigurator = driveMotor.getConfigurator();
    this.driveRequest = new DutyCycleOut(0);
    this.driveVelocityRequest = new VelocityVoltage(0);
    this.driveVelocityRequest.Slot = 0;
    this.drivePIDConfigs = new Slot0Configs();

    /* DRIVE TALONFX CONFIGS */
    TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();
    driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDriveMotorDeadband;
    driveMotorConfigs.Slot0.kP = ModuleConstants.kPDrive;
    driveMotorConfigs.Slot0.kI = ModuleConstants.kIDrive;
    driveMotorConfigs.Slot0.kD = ModuleConstants.kDDrive;
    driveMotorConfigs.Slot0.kV = ModuleConstants.kVDrive;
    // driveMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
    // driveMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
    driveMotorConfigs.MotorOutput.Inverted =
        invertDriveMotor ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    // driveMotorConfigs.CurrentLimits.StatorCurrentLimit = 40;
    driveConfigurator.apply(driveMotorConfigs);

    /* TURN SPARKMAX CONFIGS */
    this.sparkConfig = new SparkMaxConfig();

    sparkConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).openLoopRampRate(0.2)
        .inverted(invertTurningMotor);
    turnMotor.configure(sparkConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    this.brakeRequest = new NeutralOut();

    /* TURN PID CONFIGS */
    this.turnPIDController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning,
        ModuleConstants.kDTurning);

    turnPIDController.enableContinuousInput(0, 360); // Originally was -pi to pi
    turnPIDController.setTolerance(.005);

    /* CANCODER CONFIGS */
    this.invertTurningEncoder = CANCoderReversed;

    this.canCoder = new CANcoder(CANCoderId, ModuleConstants.kCANivoreName);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    config.MagnetSensor.SensorDirection =
        invertTurningEncoder ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    canCoder.getConfigurator().apply(config);

    this.encoderOffset = encoderOffset;
  }

  public void refreshPID(double kPDrive, double kIDrive, double kDDrive, double kVDrive,
      double kPTurning, double kITurning, double kDTurning) {
    turnPIDController.setPID(kPTurning, kITurning, kDTurning);

    driveConfigurator.refresh(drivePIDConfigs);
    drivePIDConfigs.kP = kPDrive;
    drivePIDConfigs.kI = kIDrive;
    drivePIDConfigs.kD = kDDrive;
    drivePIDConfigs.kV = kVDrive;
    driveConfigurator.apply(drivePIDConfigs);
  }

  public void stop() {
    driveMotor.setControl(brakeRequest);
    turnMotor.set(0);

    this.desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(getTurningPosition()));
  }

  public SwerveModuleState getDesiredState() {
    return this.desiredState;
  }

  public double closetAngle(double current, double target) {
    double direction = Math.signum(target - current);
    double changes = Math.abs(target - current);

    if (Math.abs(current - target) > 180) {
      changes = Math.abs(360 - changes);
      direction *= -1;
    }
    return changes * direction;
  }

  public void run() {
    // desiredState = SwerveModuleState.optimize(desiredState,
    // Rotation2d.fromRadians(getTurningPosition()));
    desiredAngle = desiredState.angle.getDegrees();

    double velocity = desiredState.speedMetersPerSecond / ModuleConstants.kMetersPerRevolution
        / ModuleConstants.kDriveMotorGearRatio; // Convert MPS to RPS

    // double velocity = desiredState.speedMetersPerSecond /
    // ModuleConstants.kDriveTicksPer100MsToMetersPerSec /
    // ModuleConstants.kDriveMotorGearRatio;
    this.desiredVelocity = velocity;

    double trackedCurrentAngle = getTurningPositionDegreesWithOffset();

    if (Math.abs(closetAngle(trackedCurrentAngle, desiredAngle)) > 90) {
      if (desiredAngle > 180) {
        desiredAngle -= 180;
      } else {
        desiredAngle += 180;
      }
      desiredVelocity *= -1;
    }

    if (Robot.isSimulation()) {
      var talonFxSim = driveMotor.getSimState();
      talonFxSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      var motorVoltage = talonFxSim.getMotorVoltageMeasure();

      sim_driveMotor.setInputVoltage(motorVoltage.in(Volts));

      talonFxSim.setRawRotorPosition(
          sim_driveMotor.getAngularPosition().times(ModuleConstants.kDriveMotorGearRatio));
      talonFxSim.setRotorVelocity(
          sim_driveMotor.getAngularVelocity().times(ModuleConstants.kDriveMotorGearRatio));

    }

    turnMotor.set(turnPIDController.calculate(trackedCurrentAngle, desiredAngle));
    if (Math.abs(velocity) < 0.001) {
      driveMotor.setControl(brakeRequest);

    } else if (this.velocityControl) {
      driveMotor.setControl(driveVelocityRequest.withVelocity(desiredVelocity).withSlot(0));
      this.currentPercent = 0;

    } else {
      this.currentPercent = desiredVelocity / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
      this.driveRequest.Output = currentPercent;
      driveMotor.setControl(this.driveRequest);
    }

  }

  public void resetDesiredAngle() {
    this.desiredAngle = 0;
  }

  // ****************************** GETTERS ******************************/

  /**
   * Get the distance travelled by the motor in meters
   * 
   * @return Distance travelled by motor (in meters)
   */
  public double getDrivePosition() {
    return driveMotor.getRotorPosition().getValue().in(Units.Rotations)
        * ModuleConstants.kMetersPerRevolution * ModuleConstants.kDriveMotorGearRatio;
  }

  public double getDrivePositionTicks() {
    return driveMotor.getRotorPosition().getValue().in(Units.Rotations) * 2048;
  }

  /**
   * Get the turning motor's CANCoder's angle
   * 
   * @return Angle in radians
   */
  public double getTurningPosition() {
    double turningPosition = Math.toRadians(getTurningPositionDegrees());
    return turningPosition;
  }

  /**
   * Get the turning motor's CANCoder's angle
   * 
   * @return Angle in degrees
   */
  public double getTurningPositionDegrees() {
    double turningPosition = canCoder.getAbsolutePosition().getValue().in(Units.Degrees);
    return turningPosition;
  }

  public double getTurningPositionDegreesWithOffset() {
    double angle = getTurningPositionDegrees() - encoderOffset;

    if (angle > 360) {
      angle = (angle - 360);
    }

    else if (angle <= 0) {
      angle = 360 + angle;
    }

    return angle;
  }

  /**
   * Get the velocity of the drive motor
   * 
   * @return Velocity of the drive motor (in meters / sec)
   */
  public double getDriveVelocity() {
    return driveMotor.getRotorVelocity().getValue().in(Units.RevolutionsPerSecond)
        * ModuleConstants.kMetersPerRevolution * ModuleConstants.kDriveMotorGearRatio;
  }

  /**
   * Get the velocity of the drive motor
   * 
   * @return Velocity of the drive motor (in meters / sec)
   */
  public double getDriveVelocityRPS() {
    return driveMotor.getRotorVelocity().getValue().in(Units.RotationsPerSecond);
  }

  /**
   * Get the velocity of the turning motor
   * 
   * @return Velocity of the turning motor (in radians / sec)
   */
  public double getTurningVelocity() {
    double turnVelocity = Math.toRadians(getTurningVelocityDegrees());
    return turnVelocity;
  }

  /**
   * Get the velocity of the turning motor
   * 
   * @return Velocity of the turning motor (in degrees / sec)
   */
  public double getTurningVelocityDegrees() {
    double turnVelocity = canCoder.getVelocity().getValue().in(Units.RotationsPerSecond);
    return turnVelocity;
  }

  /**
   * Return the current state (velocity and rotation) of the Swerve Module
   * 
   * @return This Swerve Module's State
   */
  public SwerveModuleState getState() {
    currState.speedMetersPerSecond = getDriveVelocity();
    currState.angle = Rotation2d.fromRadians(Math.toRadians(getTurningPositionDegreesWithOffset()));
    return currState;
  }

  public SwerveModulePosition getPosition() {
    currPosition.distanceMeters = -getDrivePosition(); // Dunno why but this fix everything
    // currPosition.angle = Rotation2d.fromRadians(getTurningPosition());
    currPosition.angle = Rotation2d.fromDegrees(getTurningPositionDegreesWithOffset());

    return currPosition;
    // return new SwerveModulePosition(getDrivePosition(), new
    // Rotation2d(getTurningPosition()));
  }

  public TalonFXSimState getTalonFXSimState() {
    return driveMotor.getSimState();
  }

  public DCMotorSim getDriveMotorSim() {
    return sim_driveMotor;
  }

  // ****************************** SETTERS ******************************/

  /**
   * Set the desired state of the Swerve Module and move towards it
   * 
   * @param state The desired state for this Swerve Module
   */
  public void setDesiredState(SwerveModuleState state, boolean withVelocityControl) {
    this.velocityControl = withVelocityControl;
    setDesiredState(state);
  }

  /**
   * Set the desired state of the Swerve Module and move towards it
   * 
   * @param state The desired state for this Swerve Module
   */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      state.speedMetersPerSecond = 0;
    }
    this.desiredState = state;
  }

  public void toggleVelocityControl(boolean velocityControlOn) {
    this.velocityControl = velocityControlOn;
  }

  public void initShuffleboard(int level) {
    // Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal
    if (level == 0) {
      return;
    }
    int moduleId = driveMotorID;
    ShuffleboardTab tab = Shuffleboard.getTab("Module " + moduleId);

    switch (level) {
      case 0:
        break;
      case 1:

        tab.addNumber("Turn percent (motor controller)", () -> turnMotor.getAppliedOutput());
        tab.addNumber("Turn percent (current)", () -> this.currentTurnPercent);
      case 2:
        tab.addNumber("Turn Motor Current", () -> turnMotor.getAppliedOutput());
        tab.addNumber("Drive Motor Voltage", () -> (driveMotor.getDutyCycle().getValue().intValue()
            * driveMotor.getSupplyVoltage().getValue().in(Units.Volts)));
        tab.addNumber("Drive percent (motor controller)",
            () -> driveMotor.getDutyCycle().getValue());
        tab.addNumber("Drive percent (current)", () -> this.currentPercent);

        tab.addNumber("Drive ticks", this::getDrivePositionTicks);
        tab.addNumber("Turn real angle", this::getTurningPositionDegrees);
        tab.addNumber("Turn angle", this::getTurningPositionDegreesWithOffset);
        tab.addNumber("Turn angle percent", () -> turnMotor.getAppliedOutput());
        tab.addNumber("Desired Angle", () -> desiredAngle);
        tab.addNumber("Angle Difference", () -> desiredAngle - currentAngle);
      case 3:
        tab.addNumber("Drive Motor Current",
            () -> driveMotor.getSupplyCurrent().getValue().in(Units.Amps));
        tab.addNumber("Module Velocity", this::getDriveVelocity);
        tab.addNumber("Module Velocity RPS", this::getDriveVelocityRPS);
        tab.addNumber("Desired Velocity", () -> this.desiredVelocity);
        tab.addBoolean("Velocity Control", () -> this.velocityControl);
        tab.addString("Error Status", () -> driveMotor.getFaultField().getName());
        // tab.addRaw("Desired State", this::getDesiredState);
        ;
        break;
      default:
        break;
    }

  }

  // Dumb Function but whatever
  public void setBreak(boolean driveBreaking, boolean turningBreaking) {
    NeutralModeValue mode = (driveBreaking ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    var config = new SparkMaxConfig();

    if (turningBreaking) {
      config.idleMode(IdleMode.kBrake);
    } else {
      config.idleMode(IdleMode.kCoast);
    }

    MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
    this.driveConfigurator.refresh(driveConfigs);
    driveConfigs.NeutralMode = mode;
    this.driveConfigurator.apply(driveConfigs);

    turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

  }

}
