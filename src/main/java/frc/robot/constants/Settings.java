/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import frc.robot.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Routine {
        public enum Mechanism {
            SWERVE_TURN,
            SWERVE_DRIVE,
            TANK_DRIVE,
            FLYWHEEL,     // must use conversion of 60 in SysID for velocity
            ELEVATOR,
            SINGLE_JOINTED_ARM,
            DOUBLE_JOINTED_ARM_JOINT_ONE,
            DOUBLE_JOINTED_ARM_JOINT_TWO
        }

        public Mechanism ROUTINE = Mechanism.SWERVE_DRIVE;
    }

    public interface Flywheel {
        double POSITION_CONVERSION = 1;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
    }

    public interface Elevator {
        double POSITION_CONVERSION = 1;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
    }

    public interface Arm {
        public interface SingleJointed {
            double POSITION_CONVERSION = 1;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }

        public interface DoubleJointed {
            public interface JointOne {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

                SmartNumber kP = new SmartNumber("Double Jointed/Joint One/kP", 1);
                SmartNumber kI = new SmartNumber("Double Jointed/Joint One/kI", 0);
                SmartNumber kD = new SmartNumber("Double Jointed/Joint One/kD", 0);

                SmartNumber kS = new SmartNumber("Double Jointed/Joint One/kS", 0);
                SmartNumber kV = new SmartNumber("Double Jointed/Joint One/kV", 0);
                SmartNumber kA = new SmartNumber("Double Jointed/Joint One/kA", 0);
                SmartNumber kG = new SmartNumber("Double Jointed/Joint One/kG", 0);
            }

            public interface JointTwo {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;

                SmartNumber kP = new SmartNumber("Double Jointed/Joint Two/kP", 1);
                SmartNumber kI = new SmartNumber("Double Jointed/Joint Two/kI", 0);
                SmartNumber kD = new SmartNumber("Double Jointed/Joint Two/kD", 0);

                SmartNumber kS = new SmartNumber("Double Jointed/Joint Two/kS", 0);
                SmartNumber kV = new SmartNumber("Double Jointed/Joint Two/kV", 0);
                SmartNumber kA = new SmartNumber("Double Jointed/Joint Two/kA", 0);
                SmartNumber kG = new SmartNumber("Double Jointed/Joint One/kG", 0);
            }
        }
    }

    public interface TankDrive {
        double POSITION_CONVERSION = 1;
        double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
    }

    public interface Swerve {

        String CANBUS = "*";

        double WIDTH = Units.inchesToMeters(26);
        double LENGTH = Units.inchesToMeters(26);

        SmartNumber MODULE_VELOCITY_DEADBAND =
                new SmartNumber("Swerve/Module velocity deadband (m per s)", 0.02);
        SmartNumber MAX_MODULE_SPEED =
                new SmartNumber("Swerve/Maximum module speed (m per s)", 5.06);
        SmartNumber MAX_MODULE_TURN =
                new SmartNumber("Swerve/Maximum module turn (rad per s)", 6.28);

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 6.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.15);

            SmartNumber kS = new SmartNumber("Swerve/Turn/kS", 0.44076);
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.0056191);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.00042985);
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.00019162);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.2021);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 0.0017542);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.00013496);
        }

        public interface Motion {

            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-153.632812 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(147.919922 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(73.125 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-2.02184 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 6.12;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO / 1000;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 21.4285714286;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
    }
}
