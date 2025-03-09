// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShoulderConstants.SHOULDER_MOTOR_SPEED;

import java.util.function.BooleanSupplier;

public class ShoulderSubsystem extends SubsystemBase {

  public enum shoulderPosition {
    START, L1, L2, L3, L4, CORAL_STATION, GROUND, UPPER_ALGAE, LOWER_ALGAE
  }

  shoulderPosition myVAR = shoulderPosition.START;

  DutyCycleEncoder absoluteEncoder;

  TalonFX shoulderMotor;
  public PIDController shoulderPidController = new PIDController(0.02, 0, 0.001);

  // 1000:1 GEAR RATIO

  /**
   * Creates a new ShoulderSubsystem.
   */
  public ShoulderSubsystem() {

    absoluteEncoder = new DutyCycleEncoder(1);
    shoulderMotor = new TalonFX(50, "rio");
    shoulderPidController.setTolerance(0.6, 0.005);
    zeroShoulderEncoder();

  }

  //double correctedAbsoluteEncoder;

  // public void absoluteEncoderCorrector() {
  //   if (absoluteEncoder.get() <= 0.1 && absoluteEncoder.get() >= 0.0) {
  //     if (getShoulderEncoder() <= XXX && getShoulderEncoder() >= XXX) {
  //       correctedAbsoluteEncoder = absoluteEncoder.get() + 1;
  //     } else {
  //       correctedAbsoluteEncoder = absoluteEncoder.get();
  //     }

  //   }
  // }

  public void shoulderMotorFRW() {
    shoulderMotor.set(SHOULDER_MOTOR_SPEED);
  }

  public void reset(BooleanSupplier elevatorInEnoughSupplier){
    if(elevatorInEnoughSupplier.getAsBoolean()){
      this.set(shoulderPidController.calculate(this.getShoulderEncoder(), 0));
    }
  }

  public void shoulderMotorBCK() {
    shoulderMotor.set(-SHOULDER_MOTOR_SPEED);
  }

  public void shoulderMotorOFF() {
    shoulderMotor.set(0);
  }

  public double getShoulderEncoder() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }

  public void zeroShoulderEncoder() {
    shoulderMotor.setPosition(0);
  }

  public void shoulderMove() {
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void shoulderSet(double speed, double direction) {
    shoulderMotor.set(speed * direction);
  }

  public void moveShoulderL1() {
    /*
     * This moves the shoulder to the position of L1 on the Reef and updates the
     * string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.L1;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderL2() {
    /*
     * This moves the shoulder to the position of L2 on the Reef and updates the
     * string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.L2;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderL3() {
    /*
     * This moves the shoulder to the position of L3 on the Reef and updates the
     * string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.L3;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderL4() {
    /*
     * This moves the shoulder to the position of L4 on the Reef and updates the
     * string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.L4;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderCoralStation() {
    /*
     * This moves the shoulder to the position of the Coral Station to intake a
     * peice and updates the string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.CORAL_STATION;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderGround() {
    /*
     * This moves the shoulder to the lowest possible position and updates the
     * string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.GROUND;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderUpperAlgae() {
    /*
     * This moves the shoulder to the position of upper algae on the Reef and
     * updates the string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.UPPER_ALGAE;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveShoulderLowerAlgae() {
    /*
     * This moves the shoulder to the position of the lower algae on the Reef and
     * updates the string shoulderPosition
     */
    if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
      shoulderMotorOFF();
      myVAR = shoulderPosition.LOWER_ALGAE;
      return;
    }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shoulder-encoder-val", shoulderMotor.getPosition().getValueAsDouble());
    SmartDashboard.putString("shoulderPosition", myVAR.toString());
    SmartDashboard.putNumber("shoulder torque", getTorque());
    SmartDashboard.putNumber("shold-abs-enc", absoluteEncoder.get());
    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    shoulderMotor.set(speed);
  }

  private double getTorque() {
    return shoulderMotor.getTorqueCurrent().getValueAsDouble() * shoulderMotor.getMotorKT().getValueAsDouble();
  }

}