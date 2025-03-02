// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristTestCommand extends Command {
  WristSubsystem wristSubsystem;
  private final double speed = 0.05;
  private double direction = 1;

  /** Creates a new WristCommand. */
  public WristTestCommand(WristSubsystem wristSubsystem, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
    this.wristSubsystem = wristSubsystem;
    this.direction = direction;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.set(speed * direction);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.wristMotorOFF();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
