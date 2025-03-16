// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primary;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This command is for lining up to intake from the coral station (angle lineup
 * in robotContainer swerve object)
 * 
 */
public class L1AlgaeClearCommand extends Command {
  /** Creates a new AutoIntake. */
  private IntakeSubsystem intakeSubsystem;
  private ShoulderSubsystem shoulderSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;

  private PIDController shoulderPID = new PIDController(0, 0, 0);
  private PIDController elevatorPID = new PIDController(0, 0, 0);

  public L1AlgaeClearCommand(IntakeSubsystem intakeSubsystem, ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;

    addRequirements(intakeSubsystem, shoulderSubsystem, elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shoulderSubsystem.moveShoulderLowerAlgae();
    elevatorSubsystem.moveElevatorLowerAlgae();
    if(elevatorSubsystem.getElevatorEncoder() <= 100){
      wristSubsystem.verticalPID();
    }
    if (shoulderSubsystem.getShoulderEncoder() >= -1) {
      
    }
    intakeSubsystem.intakeMotorFWD();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSubsystem.shoulderMotorOFF();
    elevatorSubsystem.elevatorOFF();
    intakeSubsystem.intakeMotorOFF();
    wristSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
