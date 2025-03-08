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

public class JointTwoSysID extends AbstractSysID {

    private final DoubleJointed doubleJointed;
    private final SysIdRoutine jointTwoRoutine;

    public JointTwoSysID() {
        this.doubleJointed = new DoubleJointed();
        this.jointTwoRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    doubleJointed.setMode(false, true);
                                    doubleJointed.setJointTwoVoltage(voltage.in(Units.Volts));
                                },
                                (log) -> {
                                    log.motor(doubleJointed.getName())
                                            .voltage(
                                                    Units.Volts.of(
                                                            doubleJointed.getJointTwoVoltage()))
                                            .angularPosition(
                                                    Units.Radians.of(
                                                            doubleJointed.getJointTwoPosition()))
                                            .angularVelocity(
                                                    Units.RadiansPerSecond.of(
                                                            doubleJointed.getJointTwoVelocity()));
                                },
                                this));
    }

    @Override
    public Command quasistaticForward() {
        return jointTwoRoutine.quasistatic(Direction.kForward);
    }

    @Override
    public Command quasistaticReverse() {
        return jointTwoRoutine.quasistatic(Direction.kReverse);
    }

    @Override
    public Command dynamicForward() {
        return jointTwoRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Command dynamicReverse() {
        return jointTwoRoutine.dynamic(Direction.kReverse);
    }
}
