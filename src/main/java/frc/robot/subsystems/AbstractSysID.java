/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems;

import static frc.robot.constants.Settings.Routine.*;

import frc.robot.subsystems.arm.SingleJointedSysID;
import frc.robot.subsystems.arm.doublejointed.JointOneSysID;
import frc.robot.subsystems.arm.doublejointed.JointTwoSysID;
import frc.robot.subsystems.elevator.ElevatorSysID;
import frc.robot.subsystems.flywheel.FlywheelSysID;
import frc.robot.subsystems.swerve.SwerveDriveSysID;
import frc.robot.subsystems.swerve.SwerveTurnSysID;
import frc.robot.subsystems.tank.TankSysID;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSysID extends SubsystemBase {

    public static final AbstractSysID instance;

    static {
        switch (ROUTINE) {
            case SWERVE_DRIVE:
                instance = new SwerveDriveSysID();
                break;
            case SWERVE_TURN:
                instance = new SwerveTurnSysID();
                break;
            case TANK_DRIVE:
                instance = new TankSysID();
                break;
            case FLYWHEEL:
                instance = new FlywheelSysID();
                break;
            case ELEVATOR:
                instance = new ElevatorSysID();
                break;
            case SINGLE_JOINTED_ARM:
                instance = new SingleJointedSysID();
                break;
            case DOUBLE_JOINTED_ARM_JOINT_ONE:
                instance = new JointOneSysID();
                break;
            case DOUBLE_JOINTED_ARM_JOINT_TWO:
                instance = new JointTwoSysID();
                break;
            default:
                instance = null;
                break;
        }
    }

    public static AbstractSysID getInstance() {
        return instance;
    }

    public abstract Command quasistaticForward();

    public abstract Command quasistaticReverse();

    public abstract Command dynamicForward();

    public abstract Command dynamicReverse();
}
