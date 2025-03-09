// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoIntakeCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * This command is for lining up to intake from the coral station (angle lineup in robotContainer swerve object)
 * 
 */
public class AutoIntakeCommand extends Command {
  /** Creates a new AutoIntake. */
  private IntakeSubsystem intakeSubsystem;
  private ShoulderSubsystem shoulderSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  private PIDController shoulderPID = new PIDController(0, 0, 0);
  private PIDController elevatorPID = new PIDController(0, 0, 0);


  public AutoIntakeCommand(IntakeSubsystem intakeSubsystem, ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(intakeSubsystem, shoulderSubsystem, elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.elevatorIN();
    shoulderSubsystem.set(shoulderPID.calculate(shoulderSubsystem.getShoulderEncoder(), 7));
    if(shoulderSubsystem.getShoulderEncoder() >= 0){
      elevatorSubsystem.set(elevatorPID.calculate(elevatorSubsystem.getElevatorEncoder(), 50));
    }
    if((shoulderSubsystem.getShoulderEncoder() >= 0) && (elevatorSubsystem.getElevatorEncoder() >= 48)){
      intakeSubsystem.intakeGamePiece();
    }
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
