// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primary;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/**
 * This command is for lining up to intake from the coral station (angle lineup
 * in robotContainer swerve object)
 * 
 */
public class AutoPlaceCommand extends Command {
  /** Creates a new AutoIntake. */
  private IntakeSubsystem intakeSubsystem;
  private ShoulderSubsystem shoulderSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  private PIDController shoulderPID = new PIDController(0, 0, 0);
  private PIDController elevatorPID = new PIDController(0, 0, 0);

  private double initialShoulder;
  private double desiredShoulder;
  private double shoulderSpeed;
  private boolean outtake = true;

  public AutoPlaceCommand(IntakeSubsystem intakeSubsystem, ShoulderSubsystem shoulderSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(intakeSubsystem, shoulderSubsystem, elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialShoulder = shoulderSubsystem.getShoulderEncoder();
    if((initialShoulder <= ShoulderConstants.L2_SETPOINT + 10) && (initialShoulder >= ShoulderConstants.L2_SETPOINT - 10)) { //checks if the shoulder is in a range of 2 away from the setpoint of L2
      desiredShoulder = initialShoulder - 40; //Setpoint is 30 under where it was originally
      shoulderSpeed = -0.6;
      outtake = true;
    } else if(elevatorSubsystem.getElevatorEncoder() >= 200){ //checks if the elevator is really far out, indicating L4
      desiredShoulder = initialShoulder - 14;
      shoulderSpeed = -0.15;
      outtake = false;
    } else if((initialShoulder <= ShoulderConstants.L1_SETPOINT + 10) && (initialShoulder >= ShoulderConstants.L1_SETPOINT - 10)){
      desiredShoulder = initialShoulder;
      shoulderSpeed = 0;
      outtake = true;

    }else{ //anything else, so usually L3
      desiredShoulder = initialShoulder - 25;
      shoulderSpeed = -0.6;
      outtake = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shoulderSubsystem.getShoulderEncoder() > desiredShoulder){
        shoulderSubsystem.set(shoulderSpeed);
    } else{
        shoulderSubsystem.set(0);
        if(outtake){
          intakeSubsystem.ejectGamePiece();
        }
        elevatorSubsystem.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSubsystem.shoulderMotorOFF();
    elevatorSubsystem.elevatorOFF();
    intakeSubsystem.intakeMotorOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevatorSubsystem.getElevatorEncoder() <= 10 && shoulderSubsystem.getShoulderEncoder() <= desiredShoulder){
      return true;
    }
    return false;
  }
}
