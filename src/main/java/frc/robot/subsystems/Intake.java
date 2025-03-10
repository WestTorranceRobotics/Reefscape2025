// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
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
  private final FlywheelSim sim_intake;

  // this is kinda arbitrary right now i'm not gonna lie but
  // i'll do measurements later. this sim doesn't rlly matter anyways rn
  private final double kMomentOfInertia = 5;

  // the MOTOR needs to turn twice for the INTAKE to turn once.
  private final double kGearRatio = 2;

  /** Target velocity in rotations per minute. */
  private double targetVelocity;

  /** Creates a new Intake. */
  public Intake(boolean invert) {

    // MOTOR CONFIG
    intakeMotor = new SparkMax(2, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.smartCurrentLimit(50);
    config.idleMode(IdleMode.kBrake);
    config.openLoopRampRate(0.2);
    config.inverted(invert);

    config.closedLoop.maxMotion.allowedClosedLoopError(0.2);
    config.closedLoop.pidf(0.001, 0, 0, 1);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    targetVelocity = 0;

    // SIMULATION CONFIG
    sim_gearbox = DCMotor.getNEO(1);
    sim_intakeMotor = new SparkMaxSim(intakeMotor, sim_gearbox);

    sim_intake = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(sim_gearbox, kMomentOfInertia, kGearRatio),
        sim_gearbox);
  }

  /**
   * Runs intake at a given velocity in rotations per minute.
   * 
   * @param speed { Target velocity in rotations per minute.
   */
  public void setIntakeSpeed(double speed) {
    targetVelocity = speed * 2; // gear ratio is 2:1.
    // the MOTOR needs to turn twice for the INTAKE to turn once.

    intakeMotor.getClosedLoopController().setReference(targetVelocity,
        ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  public void directSetSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Sets speed of intake to zero.
   */
  public void stop() {
    targetVelocity = 0;
    // intakeMotor.getClosedLoopController().setReference(targetVelocity,
    // ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    intakeMotor.set(0);
  }

  // public Command c_setIntakeSpeedCommand(int target_rpm) {
  //   return Commands.runOnce(() -> {
  //     intakeMotor.set(-0.2);
  //     // setIntakeSpeed(target_rpm);
  //   }, this);
  // }

  public Command c_directSetIntakeSpeedCommand(double speed) {
    return Commands.runOnce(() -> {
      intakeMotor.set(speed);
    }, this);
  }

  public Command c_stopCommand() {
    return Commands.runOnce(() -> {
      stop();
    }, this);
  }

  public void initShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    tab.addNumber("Intake target velocity", () -> targetVelocity);

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
      sim_intake.setInput(sim_intakeMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
      sim_intake.update(0.02);

      sim_intakeMotor.iterate(targetVelocity / 60, RoboRioSim.getVInVoltage(), 0.02);

      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(sim_intake.getCurrentDrawAmps()));

    }
  }
}
