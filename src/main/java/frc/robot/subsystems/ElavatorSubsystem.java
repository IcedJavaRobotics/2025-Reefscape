// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElavatorSubsystem extends SubsystemBase {
  /** Creates a new ElavatorSubsystem. */

  TalonFX elavatorMotor;

  public ElavatorSubsystem() {

    this.elavatorMotor = new TalonFX(51, "rio");
  }

  public void elavatorOUT() {

    elavatorMotor.set(Constants.ELAVATOR_MOTOR_SPEED);
  }

  public void elavatorIN() {

    elavatorMotor.set(-Constants.ELAVATOR_MOTOR_SPEED);
  }

  public void elavatorStop() {
    elavatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
