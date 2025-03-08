/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.arm;

import frc.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SingleJointedSysID extends AbstractSysID {

    private final SingleJointed singleJointed;
    private final SysIdRoutine singleJointedRoutine;

    public SingleJointedSysID() {
        this.singleJointed = new SingleJointed();
        this.singleJointedRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    singleJointed.setVoltage(voltage.in(Units.Volts));
                                },
                                (log) -> {
                                    log.motor(singleJointed.getName())
                                            .voltage(Units.Volts.of(singleJointed.getVoltage()))
                                            .angularPosition(
                                                    Units.Radians.of(singleJointed.getPosition()))
                                            .angularVelocity(
                                                    Units.RadiansPerSecond.of(
                                                            singleJointed.getVelocity()));
                                },
                                this));
    }

    @Override
    public Command quasistaticForward() {
        return singleJointedRoutine.quasistatic(Direction.kForward);
    }

    @Override
    public Command quasistaticReverse() {
        return singleJointedRoutine.quasistatic(Direction.kReverse);
    }

    @Override
    public Command dynamicForward() {
        return singleJointedRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Command dynamicReverse() {
        return singleJointedRoutine.dynamic(Direction.kReverse);
    }
}
