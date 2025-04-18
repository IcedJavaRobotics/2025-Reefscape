// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ActuatorSubsystem extends SubsystemBase {

  private Servo climberActuator;

  /** Creates a new ActuatorSubsystem. */
  public ActuatorSubsystem() {
    climberActuator = new Servo(0);
    // setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    climberActuator.setBoundsMicroseconds((int) (1000 + ActuatorConstants.length), (int) (1000 + ActuatorConstants.length * .8),
        (int) (1000 + ActuatorConstants.length * .5), (int) (1000 + ActuatorConstants.length * .2), 1000);
  }// (1500, 1400, 1250, 1200, 1000);

  public void ActuatorOUT() {
    climberActuator.setSpeed(1);

  }

  public void ActuatorIN() {
    climberActuator.setSpeed(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber act", climberActuator.get());
  }
}
