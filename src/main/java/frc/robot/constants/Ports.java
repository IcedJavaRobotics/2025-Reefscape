/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {

    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Flywheel {
        int MOTOR = 0;
    }

    public interface Elevator {
        int MOTOR = 1;
        int MOTOR2 = 2;
    }

    public interface Arm {
        int JOINT_ONE = 3;
        int JOINT_TWO = 4;
    }

    public interface TankDrive {
        int FRONT_LEFT_MOTOR = 5;
        int BACK_LEFT_MOTOR = 6;
        int FRONT_RIGHT_MOTOR = 7;
        int BACK_RIGHT_MOTOR = 8;
    }

    public interface Swerve {
        public interface BackLeft {
            int DRIVE = 10;
            int TURN = 11;
            int ENCODER = 3;
        }

        public interface BackRight { 
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 1;
        }
        
        public interface FrontRight { 
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 4;
        }
        
        public interface FrontLeft {
            int DRIVE = 16;
            int TURN = 17;
            int ENCODER = 2;
        }
    }
}
