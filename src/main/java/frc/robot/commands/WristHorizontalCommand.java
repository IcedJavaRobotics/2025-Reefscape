// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristHorizontalCommand extends Command {
    WristSubsystem wristSubsystem;

    PIDController wristPID = new PIDController(0.03, 0, 0);

    /** Creates a new WristCommand. */
    public WristHorizontalCommand(WristSubsystem wristSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(wristSubsystem);
        this.wristSubsystem = wristSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // 100:1 ratio
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wristSubsystem.set(wristPID.calculate(wristSubsystem.getEncoder(), 0));
        // wristSubsystem.wristRotate();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.wristMotorOFF();
        wristSubsystem.toggleDirection();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }
}