// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.moveToCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveLowerAlgaeCommand extends Command {

    ShoulderSubsystem shoulderSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    WristSubsystem wristSubsystem;
    IntakeSubsystem intakeSubsystem;
    PIDController wristPID = new PIDController(0.03, 0, 0);

    /**
     * Creates a new MoveUpperAlgaeCommand.
     */
    public MoveLowerAlgaeCommand(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shoulderSubsystem, elevatorSubsystem, wristSubsystem);
        this.shoulderSubsystem = shoulderSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wristSubsystem.set(wristPID.calculate(wristSubsystem.getEncoder(), 0));
        shoulderSubsystem.moveShoulderLowerAlgae();
        elevatorSubsystem.moveElevatorLowerAlgae();

        if (elevatorSubsystem.getElevatorEncoder() <= 100) {
            wristSubsystem.verticalPID();
        }
        if (shoulderSubsystem.getShoulderEncoder() >= -1) {

        }
        intakeSubsystem.intakeMotorBKWD(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.wristMotorOFF();
        elevatorSubsystem.elevatorOFF();
        intakeSubsystem.intakeMotorOFF();
        shoulderSubsystem.shoulderMotorOFF();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}