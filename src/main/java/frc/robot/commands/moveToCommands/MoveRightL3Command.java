// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.moveToCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveRightL3Command extends Command {

    ShoulderSubsystem shoulderSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    WristSubsystem wristSubsystem;
    PIDController wristPID = new PIDController(0.03, 0, 0);
    PIDController elevatorPID = new PIDController(0,0,0);
    boolean reset = false;
    /**
     * Creates a new MoveRightL3Command.
     */
    public MoveRightL3Command(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.reset = false;
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
            shoulderSubsystem.moveShoulderL3();
            elevatorSubsystem.moveElevatorL3();
        
        SmartDashboard.putBoolean("resetBool", reset);
        // if (elevatorSubsystem.extensionChecker()) {
        // elevatorSubsystem.elevatorIN();
        // } else {

        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.wristMotorOFF();
        shoulderSubsystem.shoulderMotorOFF();
        elevatorSubsystem.elevatorOFF();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(wristSubsystem.getEncoder() <= 3 && 
        (shoulderSubsystem.getShoulderEncoder() >= (Constants.ShoulderConstants.L3_SETPOINT-3) && (shoulderSubsystem.getShoulderEncoder() <= (Constants.ShoulderConstants.L3_SETPOINT+3)))
         && elevatorSubsystem.getElevatorEncoder() >= (Constants.ElevatorConstants.L3_SETPOINT -4) && elevatorSubsystem.getElevatorEncoder() <= (Constants.ElevatorConstants.L3_SETPOINT + 4)
         ){
            return true;
        }
        return false;
    }
}