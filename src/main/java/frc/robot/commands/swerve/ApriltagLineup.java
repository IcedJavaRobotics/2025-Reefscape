// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApriltagLineup extends Command {

  private SwerveSubsystem swerveSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private ChassisSpeeds driveSpeeds;

  private PIDController rotationPID = new PIDController(0.01, 0, 0);
  private PIDController translationPID = new PIDController(0.01, 0, 0);

  /** Creates a new ApriltagLineup. */
  public ApriltagLineup(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(swerveSubsystem, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveSpeeds = new ChassisSpeeds(translationPID.calculate(0, limelightSubsystem.getTx()), 0, rotationPID.calculate(limelightSubsystem.getReefHeading(), swerveSubsystem.getSwerveDrive().getYaw().getDegrees()));
    driveSpeeds = new ChassisSpeeds(10, 10, 5);
    swerveSubsystem.driveRobotOriented(driveSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
