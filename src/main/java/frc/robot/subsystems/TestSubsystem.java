// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.WristConstants.*;

public class TestSubsystem extends SubsystemBase {

  //true is going clockwise false is counterclockwise
  TalonFX testMotor1;
  TalonFX testMotor2;
  TalonFX testMotor3;
  TalonFX testMotor4;
  /** Creates a new WristSubsystem. */
  public TestSubsystem() {
    testMotor1 = new TalonFX(12);
    testMotor1.setPosition(0);
    testMotor2 = new TalonFX(22);
    testMotor2.setPosition(0);
    testMotor3 = new TalonFX(32);
    testMotor3.setPosition(0);
    testMotor4 = new TalonFX(42);
    testMotor4.setPosition(0);
  }

  public void set(double speed){
    testMotor1.set(speed);
    testMotor2.set(speed);
    testMotor3.set(speed);
    testMotor4.set(speed);
  }

  public void stop(){
    testMotor1.set(0);
    testMotor2.set(0);
    testMotor3.set(0);
    testMotor4.set(0);
  }


 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("test-encoder-val1", testMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("test-encoder-val2", testMotor2.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("test-encoder-val3", testMotor3.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("test-encoder-val4", testMotor4.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
