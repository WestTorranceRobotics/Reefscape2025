// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;
  private final SparkMaxConfig config;

  private final DCMotor sim_gearbox;
  private final SparkMaxSim sim_intakeMotor;

  private final FlywheelSim intakeSim;

  /**
   * The intake's gear ratio is 2:1. Ihe motor needs to turn twice for the intake to turn once.
   */
  private final double kGearRatio = 2;

  /** Target velocity in rotations per minute. */
  private double targetRpm;

  /** Creates a new Intake. */
  public Intake(boolean inverted) {

    // MOTOR CONFIG
    intakeMotor = new SparkMax(2, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.smartCurrentLimit(50);
    config.idleMode(IdleMode.kBrake);
    config.openLoopRampRate(0.2);
    config.inverted(inverted);

    // config.closedLoop.maxMotion.allowedClosedLoopError(5);
    config.closedLoop.smartMotion.allowedClosedLoopError(5);
    config.closedLoop.pidf(0.01, 0, 0, 1);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    targetRpm = 0;

    // SIMULATION CONFIG
    sim_gearbox = DCMotor.getNEO(1);
    sim_intakeMotor = new SparkMaxSim(intakeMotor, sim_gearbox);

    intakeSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(sim_gearbox, 1, 2), sim_gearbox);
  }

  /**
   * Runs intake at a given velocity in rotations per minute.
   * 
   * @param rpm Target velocity in rotations per minute.
   */
  public void setIntakeSpeed(double rpm) {
    targetRpm = rpm * kGearRatio;
    intakeMotor.getClosedLoopController()
        .setReference(targetRpm, ControlType.kSmartVelocity, ClosedLoopSlot.kSlot0);
  }

  /**
   * Directly sets the intake motor's speed to a percentage, from 1.0 to -1.0.
   * 
   * @param speed A percentage from 1.0 to -1.0.
   */
  public void directSetSpeed(double speed) {
    intakeMotor.set(speed);
  }


  /**
   * Idles intake at 2% power.
   */
  public void idle() {
    intakeMotor.set(0.02);
  }

  /**
   * Sets intake speed to zero.
   */
  public void stop() {
    targetRpm = 0;
    intakeMotor.set(0);
  }

  /**
   * Command to run intake at a given velocity in rotations per minute.
   * 
   * @param target_rpm Target velocity in rotations per minute.
   * @return An instant command setting the speed.
   */
  public Command c_setIntakeSpeedCommand(int target_rpm) {
    return Commands.runOnce(() -> {
      setIntakeSpeed(target_rpm);
    }, this);
  }

  /**
   * Directly sets the intake motor's speed to a percentage, from 1.0 to -1.0.
   * 
   * @param speed A percentage from 1.0 to -1.0.
   * @return An instantly finishing command setting the speed.
   */
  public Command c_directSetIntakeSpeedCommand(double speed) {
    return Commands.runOnce(() -> {
      directSetSpeed(speed);
    }, this);
  }

  /**
   * Idles intake at 2% power.
   */
  public Command c_idleCommand() {
    return Commands.runOnce(() -> {
      idle();
    }, this);
  }

  /**
   * Sets intake speed to zero.
   */
  public Command c_stopCommand() {
    return Commands.runOnce(() -> {
      stop();
    }, this);
  }

  /**
   * Log target and actual velocities to Shuffleboard. Call during initialization.
   */
  public void initShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    tab.addNumber("Intake target velocity", () -> targetRpm);

    if (Robot.isSimulation()) {
      tab.addNumber("Intake actual velocity", () -> sim_intakeMotor.getVelocity());
    } else {
      tab.addNumber("Intake actual velocity", () -> intakeMotor.getEncoder().getVelocity());
    }

    tab.addNumber("Intake applied output", () -> intakeMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      intakeSim.setInput(sim_intakeMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
      intakeSim.update(0.02);

      sim_intakeMotor.iterate(
          intakeSim.getAngularVelocity().in(Units.RevolutionsPerSecond) * 60, // rpm
          RoboRioSim.getVInVoltage(),
          0.02);
    }
  }
}
