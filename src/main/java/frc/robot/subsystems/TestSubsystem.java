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

  // true is going clockwise false is counterclockwise
  TalonFX testMotor;

  /** Creates a new WristSubsystem. */
  public TestSubsystem() {
    testMotor = new TalonFX(65);
    testMotor.setPosition(0);
  }

  public void set(double speed) {
    testMotor.set(speed);
  }

  public void stop() {
    testMotor.set(0);
  }

  public double getEncoder() {
    return testMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("test-encoder-val", testMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
