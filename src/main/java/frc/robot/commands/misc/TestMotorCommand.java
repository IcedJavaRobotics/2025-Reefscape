// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestMotorCommand extends Command {

  TestSubsystem testSubsystem;

  /** Creates a new WristCommand. */
  public TestMotorCommand(TestSubsystem testSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(testSubsystem);
    this.testSubsystem = testSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testSubsystem.set(0.05);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}