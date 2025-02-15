// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final int CANDLE_ID = 57;
  public static final double SHOULDER_MOTOR_SPEED = 0.5;
  public static final double INTAKE_MOTOR_SPEED = 0.1;
  public static final double DISTANCE_FROM_PIECE = 1; //This value is in inches
  public static final double INTAKE_PULSE_INTERVAL = 2.5; //This value is in seconds
  public static final double INTAKE_PULSE_LENGTH = 1; //This value is in seconds
}
