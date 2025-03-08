package frc.robot.subsystems.tank;

import frc.robot.subsystems.AbstractSysID;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class TankSysID extends AbstractSysID {

    private final TankDrive tankDrive;
    private final SysIdRoutine tankDriveRoutine;

    public TankSysID() {
        this.tankDrive = new TankDrive();
        this.tankDriveRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    tankDrive.setVoltage(voltage.in(Units.Volts));
                                },
                                (log) -> {
                                    log.motor(tankDrive.getName())
                                            .voltage(Units.Volts.of(tankDrive.getVoltage()))
                                            .linearPosition(
                                                    Units.Meters.of(tankDrive.getPosition()))
                                            .linearVelocity(
                                                    Units.MetersPerSecond.of(
                                                            tankDrive.getVelocity()));
                                },
                                this));
    }

    @Override
    public Command quasistaticForward() {
        return tankDriveRoutine.quasistatic(Direction.kForward);
    }

    @Override
    public Command quasistaticReverse() {
        return tankDriveRoutine.quasistatic(Direction.kReverse);
    }

    @Override
    public Command dynamicForward() {
        return tankDriveRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Command dynamicReverse() {
        return tankDriveRoutine.dynamic(Direction.kReverse);
    }
}
