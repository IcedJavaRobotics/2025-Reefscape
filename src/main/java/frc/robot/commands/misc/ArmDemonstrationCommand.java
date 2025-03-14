// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmDemonstrationCommand extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private ShoulderSubsystem shoulderSubsystem;

  /** Creates a new ArmDemonstrationCommand. */
  public ArmDemonstrationCommand(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem,
      double direction) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    addRequirements(elevatorSubsystem, shoulderSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderSubsystem.getShoulderEncoder();
    elevatorSubsystem.getElevatorEncoder();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
}
