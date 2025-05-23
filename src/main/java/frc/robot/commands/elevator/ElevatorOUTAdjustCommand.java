// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorOUTAdjustCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;

  double initialValue;

  /** Creates a new ElevatorOUTCommand. */
  public ElevatorOUTAdjustCommand(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialValue = elevatorSubsystem.getElevatorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.elevatorOUT();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.elevatorOFF();
    elevatorSubsystem.setOffset(elevatorSubsystem.getElevatorEncoder()+initialValue);
    //get offset value by subtracting new encoder value and the constant encoder
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}