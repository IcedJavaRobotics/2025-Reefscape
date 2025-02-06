// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestSubsystem extends SubsystemBase {

    TalonFX testMotor;
    double speedValue = 0;
  /** Creates a new TestSubsystem. */
  public TestSubsystem() {
    this.testMotor = new TalonFX(61, "CANivore-name");
  }

  public void increaseSpeed(double increase){
    speedValue += increase;
    this.testMotor.set(speedValue);
  }
  
  public void setMotor(double speed){
    this.testMotor.set(speed);
  }

  public void stopMotor(){
    this.testMotor.set(0);
  }

  public double getTorque(){
    return testMotor.getTorqueCurrent().getValueAsDouble() * testMotor.getMotorKT().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Torque", getTorque());
    SmartDashboard.putNumber("Kt", testMotor.getMotorKT().getValueAsDouble());
    SmartDashboard.putNumber("output current", testMotor.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("currentSpeed", speedValue);
  }
}
