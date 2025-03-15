// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeOutCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  boolean forceful;
  /** Creates a new IntakeOutCommand. */
  public IntakeOutCommand(IntakeSubsystem intakeSubsystem, boolean forceful) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
    this.forceful = forceful;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(forceful){
      intakeSubsystem.intakeMotorBKWD(0.7);
    } else{
      intakeSubsystem.ejectGamePiece();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intakeMotorOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}