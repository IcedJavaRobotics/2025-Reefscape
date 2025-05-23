// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.moveToCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveGroundCommand extends Command {

    ShoulderSubsystem shoulderSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    /**
     * Creates a new MoveGroundCommand.
     */
    public MoveGroundCommand(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shoulderSubsystem, elevatorSubsystem);
        this.shoulderSubsystem = shoulderSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shoulderSubsystem.moveShoulderGround();
        // if (elevatorSubsystem.extensionChecker()) {
        // elevatorSubsystem.elevatorIN();
        // } else {
        elevatorSubsystem.moveElevatorGround();
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.elevatorOFF();
        shoulderSubsystem.shoulderMotorOFF();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}