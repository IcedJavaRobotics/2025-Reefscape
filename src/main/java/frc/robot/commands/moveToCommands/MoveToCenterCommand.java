// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToCenterCommand extends Command {
  /** Creates a new MoveToCenterCommand. */
  private LimelightSubsystem limelight;
  private SwerveSubsystem swerveSubsystem;


  public MoveToCenterCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
    this.limelight = limelightSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(limelightSubsystem, swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
