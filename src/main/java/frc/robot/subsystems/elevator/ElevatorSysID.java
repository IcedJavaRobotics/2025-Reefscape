/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.elevator;

import frc.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSysID extends AbstractSysID {

    private final Elevator elevator;
    private final SysIdRoutine elevatorRoutine;

    public ElevatorSysID() {
        this.elevator = new Elevator();
        this.elevatorRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    elevator.setVoltage(voltage.in(Units.Volts));
                                },
                                (log) -> {
                                    log.motor(elevator.getName())
                                            .voltage(Units.Volts.of(elevator.getVoltage()))
                                            .linearPosition(Units.Meters.of(elevator.getPosition()))
                                            .linearVelocity(
                                                    Units.MetersPerSecond.of(
                                                            elevator.getVelocity()));
                                },
                                this));
    }

    @Override
    public Command quasistaticForward() {
        return elevatorRoutine.quasistatic(Direction.kForward);
    }

    @Override
    public Command quasistaticReverse() {
        return elevatorRoutine.quasistatic(Direction.kReverse);
    }

    @Override
    public Command dynamicForward() {
        return elevatorRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Command dynamicReverse() {
        return elevatorRoutine.dynamic(Direction.kReverse);
    }
}
