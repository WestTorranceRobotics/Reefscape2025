// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    /*
     * Elevator motor tick count to reach designated reef level for auto period (current value is a
     * placeholder)
     */
    public static final float elevatorTicks = 2000;
    /*
     * Intake motor tick count to angle coral for auto period (current value is a placeholder)
     */
    public static final float intakeTicks = 1000;
    
  }
}
