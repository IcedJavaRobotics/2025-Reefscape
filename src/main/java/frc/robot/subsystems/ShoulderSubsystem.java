// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
  TalonFX shoulderMotor;
  public PIDController shoulderPidController = new PIDController(0.02, 0, 0.001);

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() {
    shoulderMotor = new TalonFX(50, "CANivore-name");
    shoulderMotor.getPosition().getValueAsDouble();
    shoulderPidController.setTolerance(0.6, 0.005);
    zeroEncoder();
  }

  public void shoulderMotorFRW() {
    shoulderMotor.set(Constants.SHOULDER_MOTOR_SPEED);
  }

  public void shoulderMotorBCK() {
    shoulderMotor.set(-Constants.SHOULDER_MOTOR_SPEED);
  }

  public void shoulderMotorOFF() {
    shoulderMotor.set(0);
  }

  public double getEncoder() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }

  public void zeroEncoder(){
    shoulderMotor.setPosition(0);
   }

  public void shoulderMove(){
  if(shoulderMotor.getPosition().getValueAsDouble() >= 10) {
    shoulderMotorOFF();
    return;
  }

  // intakeMotor.set(-IntakeConstants.SPEED);

  shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}