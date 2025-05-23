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

public class WristSubsystem extends SubsystemBase {

  boolean wristDirection = true;
  // true is going clockwise false is counterclockwise
  TalonFX wristMotor;
  PIDController wristPID = new PIDController(0.03, 0.0005, 0);

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    wristMotor = new TalonFX(47);
    // wristMotor.getPosition().getValueAsDouble();
    //wristPID.setTolerance(0.3, 0.005);
    zeroEncoder();
  }

  public void wristRotate() {
    if (wristDirection == true) {
      if (wristMotor.getPosition().getValueAsDouble() >= WRIST_POSITION_TWO) {
        wristMotorOFF();
        return;
      }

      // intakeMotor.set(-IntakeConstants.SPEED);

      wristMotor.set(0.02);
    } else {
      if (wristMotor.getPosition().getValueAsDouble() <= WRIST_POSITION_ONE) {
        wristMotorOFF();
        return;
      }

      // intakeMotor.set(-IntakeConstants.SPEED);

      wristMotor.set(-0.02);
    }

  }

  public void verticalPID(){
    this.set(wristPID.calculate(this.getEncoder(), 25));
  }
  public void horizontalPID(){
    this.set(wristPID.calculate(this.getEncoder(), 0));
  }

  public void emergencyZero(){
    wristMotor.setPosition(0);
  }

  public void set(double speed) {
    wristMotor.set(speed);
  }

  public void toggleDirection() {
    wristDirection = !wristDirection;
  }

  public void wristMotorFRW() {
    wristMotor.set(WRIST_MOTOR_SPEED);
  }

  public void wristMotorBCK() {
    wristMotor.set(-WRIST_MOTOR_SPEED);
  }

  public void wristMotorOFF() {
    wristMotor.set(0);
  }

  public void zeroEncoder() {
    wristMotor.setPosition(0);
  }

  public double getEncoder() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist-encoder-val", wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("wristDirection", wristDirection);
    // This method will be called once per scheduler run
  }
}
