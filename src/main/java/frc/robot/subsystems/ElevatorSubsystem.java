// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  TalonFX elevatorMotor;

  public ElevatorSubsystem() {

    this.elevatorMotor = new TalonFX(51, "rio");
  }

  public void elevatorOUT() {

    elevatorMotor.set(Constants.Elevator_MOTOR_SPEED);
  }

  public void elevatorIN() {

    elevatorMotor.set(-Constants.Elevator_MOTOR_SPEED);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
