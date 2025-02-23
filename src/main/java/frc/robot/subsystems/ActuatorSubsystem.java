// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ActuatorSubsystem extends SubsystemBase {

  private Servo climberActuator;

  /** Creates a new ActuatorSubsystem. */
  public ActuatorSubsystem() {
    climberActuator = new Servo(5);
  }

  public void ActuatorOUT() {
    climberActuator.set(1);

  }

  public void ActuatorIN() {
    climberActuator.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
