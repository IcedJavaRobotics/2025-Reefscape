// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.moveToCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveRightL2Command extends Command {

    ShoulderSubsystem shoulderSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    WristSubsystem wristSubsystem;
    PIDController wristPID = new PIDController(0.03, 0, 0);

    /**
     * Creates a new MoveRightL2Command.
     */
    public MoveRightL2Command(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shoulderSubsystem, elevatorSubsystem, wristSubsystem);
        this.shoulderSubsystem = shoulderSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wristSubsystem.set(wristPID.calculate(wristSubsystem.getEncoder(), 0));
        shoulderSubsystem.moveShoulderL2();
        elevatorSubsystem.moveElevatorL2();
        // if (elevatorSubsystem.extensionChecker()) {
        // elevatorSubsystem.elevatorIN();
        // } else {
        // elevatorSubsystem.moveElevatorL2();
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.wristMotorOFF();
        elevatorSubsystem.elevatorOFF();
        shoulderSubsystem.shoulderMotorOFF();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}