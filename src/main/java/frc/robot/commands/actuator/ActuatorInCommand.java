// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actuator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ActuatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActuatorInCommand extends InstantCommand {
  ActuatorSubsystem actuatorSubsystem;

  public ActuatorInCommand(ActuatorSubsystem actuatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(actuatorSubsystem);
    this.actuatorSubsystem = actuatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("IM TRYING TO DO THE ACTUATOR IN COMMAND RAAAHHHH!!!!!!");
    actuatorSubsystem.ActuatorIN();
  }
}
