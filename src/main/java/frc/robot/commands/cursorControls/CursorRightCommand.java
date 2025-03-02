// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cursorControls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SelectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CursorRightCommand extends Command {
    SelectorSubsystem selectorSubsystem;

    /** Creates a new CursorUpCommand. */
    public CursorRightCommand(SelectorSubsystem selectorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(selectorSubsystem);
        this.selectorSubsystem = selectorSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        selectorSubsystem.cursorRight();
        System.out.println("CURSOR RIGHT");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}