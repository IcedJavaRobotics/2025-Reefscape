// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShoulderCommand extends Command {
  ShoulderSubsystem shoulderSubsystem;
  int multiplier = 1;

  /** Creates a new ShoulderCommand. */
  public ShoulderCommand(ShoulderSubsystem shoulderSubsystem, int multiplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.multiplier = multiplier;
    addRequirements(shoulderSubsystem);
    this.shoulderSubsystem = new ShoulderSubsystem();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulderSubsystem.shoulderMove(multiplier);
    System.out.println("SHOULDER COMMAND");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSubsystem.shoulderMotorOFF();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
