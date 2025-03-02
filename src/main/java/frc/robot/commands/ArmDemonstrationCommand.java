// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmDemonstrationCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private ShoulderSubsystem shoulderSubsystem;

  private double speed = 0.3;
  private double direction = 1;

  /** Creates a new ArmDemonstrationCommand. */
  public ArmDemonstrationCommand(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem,
      double direction) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.direction = direction;
    addRequirements(elevatorSubsystem, shoulderSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulderSubsystem.set(speed * direction); // 125 : 1
    elevatorSubsystem.set(getElevatorSpeed(shoulderSubsystem.getShoulderEncoder()) * direction); // 60 : 1
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSubsystem.set(0);
    elevatorSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getElevatorSpeed(double speed) {
    // return 0.1;
    return (-40.5 * Math.sin(3.141 / 180 * (speed / (115 / 45)))) / ((1000 / 60)); // GEAR
    // RATIO
  }
}
