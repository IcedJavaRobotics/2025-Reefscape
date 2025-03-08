/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.arm.doublejointed;

import frc.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class JointOneSysID extends AbstractSysID {

    private final DoubleJointed doubleJointed;
    private final SysIdRoutine jointOneRoutine;

    public JointOneSysID() {
        this.doubleJointed = new DoubleJointed();
        this.jointOneRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    doubleJointed.setMode(true, false);
                                    doubleJointed.setJointOneVoltage(voltage.in(Units.Volts));
                                },
                                (log) -> {
                                    log.motor(doubleJointed.getName())
                                            .voltage(
                                                    Units.Volts.of(
                                                            doubleJointed.getJointOneVoltage()))
                                            .angularPosition(
                                                    Units.Radians.of(
                                                            doubleJointed.getJointOnePosition()))
                                            .angularVelocity(
                                                    Units.RadiansPerSecond.of(
                                                            doubleJointed.getJointOneVelocity()));
                                },
                                this));
    }

    @Override
    public Command quasistaticForward() {
        return jointOneRoutine.quasistatic(Direction.kForward);
    }

    @Override
    public Command quasistaticReverse() {
        return jointOneRoutine.quasistatic(Direction.kReverse);
    }

    @Override
    public Command dynamicForward() {
        return jointOneRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Command dynamicReverse() {
        return jointOneRoutine.dynamic(Direction.kReverse);
    }
}
