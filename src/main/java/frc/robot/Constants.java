// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriverConstants {
    public static final int MAIN_DRIVER_PORT = 0;
    public static final int AUX_DRIVER_PORT = 1;
    public static final int DRIVER_STATION_PORT = 2;
    // public static final String DEADBAND = "0.5";
    public static final double DEADBAND = 0.02;
    public static final double MAX_SPEED = Units.feetToMeters(4.5);
  }

  public static class CandleConstants {
    public static final int CANDLE_ID = 57;
  }

  public static class WristConstants {
    public static final double WRIST_MOTOR_SPEED = 0.2;
    public static final double WRIST_POSITION_TWO = 5;
    public static final double WRIST_POSITION_ONE = 0;
  }

  public static class ShoulderConstants {
    public static final double SHOULDER_MOTOR_SPEED = 0.2;
  }

  public static class IntakeConstants {
    public static final double INTAKE_MOTOR_SPEED = 1;
    public static final double INTAKE_OUT_MOTOR_SPEED = 0.7;
    public static final double INTAKE_OUT_MOTOR_SLOW_SPEED = 0.3;
    public static final double DISTANCE_FROM_PIECE = 1; // This value is in inches
    public static final double INTAKE_PULSE_INTERVAL = 2.5; // This value is in seconds
    public static final double INTAKE_PULSE_LENGTH = 0; // This value is in seconds
  }

 public static class ActuatorConstants {
  public static final int length = 100; // Servo Length
 }


  public static class ElevatorConstants {
    public static final double Elevator_MOTOR_SPEED = 0.2;
  }

  public static final class LimelightConstants {
    /** upward angle of limelight camera [degrees] */
    public static final double LIMELIGHT_ANGLE = 3.0;
    /** distance from limelight lens from floor [inches] */
    public static final double LIMELIGHT_HEIGHT = 18.5;
    /** distance from apriltag to floor(bottom of tag) [inches] */
    public static final double APRILTAG_HEIGHT = 14.25;
    /**
     * distance from apriltag to floor but its the double substation(bottom of tag)
     * [inches]
     */
    public static final double APRILTAG_DOUBLE_SUBSTATION_HEIGHT = 23.375;
  }

}
