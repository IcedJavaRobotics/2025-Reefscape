// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShoulderConstants.*;

import java.util.function.BooleanSupplier;

public class ShoulderSubsystem extends SubsystemBase {

  public enum shoulderPosition {
    START, L1, L2, L2_SCORE, L3, L3_SCORE, L4, CORAL_STATION, GROUND, UPPER_ALGAE, LOWER_ALGAE
  }

  public shoulderPosition myVAR = shoulderPosition.START;

  DutyCycleEncoder absoluteEncoder;

  TalonFX shoulderMotor;
  public PIDController shoulderPidController = new PIDController(0.02, 0, 0.001);

  private double shoulderOffset = 0;

  // 1000:1 GEAR RATIO

  /**
   * Creates a new ShoulderSubsystem.
   */
  public ShoulderSubsystem() {

    absoluteEncoder = new DutyCycleEncoder(ABSOLUTE_ENCODER_CHANNEL);
    shoulderMotor = new TalonFX(SHOULDER_MOTOR_ID, "rio");
    shoulderPidController.setTolerance(0.6, 0.005);
    zeroShoulderEncoder();

  }

  // double correctedAbsoluteEncoder;

  // public void absoluteEncoderCorrector() {
  // if (absoluteEncoder.get() <= 0.1 && absoluteEncoder.get() >= 0.0) {
  // if (getShoulderEncoder() <= XXX && getShoulderEncoder() >= XXX) {
  // correctedAbsoluteEncoder = absoluteEncoder.get() + 1;
  // } else {
  // correctedAbsoluteEncoder = absoluteEncoder.get();
  // }

  // }
  // }

  public void shoulderMotorFRW() {
    shoulderMotor.set(SHOULDER_MOTOR_SPEED);
  }

  public void reset(BooleanSupplier elevatorInEnoughSupplier) {
    if (elevatorInEnoughSupplier.getAsBoolean()) {
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
  public double getAbsoluteEncoder(){
    return absoluteEncoder.get();
  }

  public void setOffset(double newOffset){
    this.shoulderOffset = newOffset;
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
    // if (shoulderMotor.getPosition().getValueAsDouble() >= -78.788) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.L1;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), (L1_SETPOINT + shoulderOffset)));
  }

  public void moveShoulderL2() {
    /*
     * This moves the shoulder to the position of L2 on the Reef and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= -32.677) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.L2;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), (L2_SETPOINT + shoulderOffset)));
  }

  public void moveShoulderL2Score() {
    /*
     * This moves the shoulder to the position of L2 on the Reef and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= -61.86) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.L2_SCORE;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), -61.86));
  }

  public void moveShoulderL3() {
    /*
     * This moves the shoulder to the position of L3 on the Reef and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= 6.926) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.L3;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), L3_SETPOINT+shoulderOffset));
  }

  public void moveShoulderL3Score() {
    /*
     * This moves the shoulder to the position of L3 on the Reef and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= -14.86) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.L3_SCORE;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), -14.86));
  }

  public void moveShoulderL4() {
    /*
     * This moves the shoulder to the position of L4 on the Reef and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.L4;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), L4_SETPOINT+shoulderOffset));
  }

  public void moveShoulderCoralStation() {
    /*
     * This moves the shoulder to the position of the Coral Station to intake a
     * peice and updates the string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= 2) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.CORAL_STATION;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), CORAL_STATION_SETPOINT+shoulderOffset));
  }

  public void moveShoulderGround() {
    /*
     * This moves the shoulder to the lowest possible position and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= -174.5) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.GROUND;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), GROUND_SETPOINT+shoulderOffset));
  }

  public void moveShoulderGroundVertical() {
    /*
     * This moves the shoulder to the lowest possible position and updates the
     * string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= -174.5) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.GROUND;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), GROUND_VERTICAL_SETPOINT+shoulderOffset));
  }

  public void moveShoulderUpperAlgae() {
    /*
     * This moves the shoulder to the position of upper algae on the Reef and
     * updates the string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.UPPER_ALGAE;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), UPPER_ALGAE_SETPOINT+shoulderOffset));
  }

  public void moveShoulderLowerAlgae() {
    /*
     * This moves the shoulder to the position of the lower algae on the Reef and
     * updates the string shoulderPosition
     */
    // if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
    // shoulderMotorOFF();
    myVAR = shoulderPosition.LOWER_ALGAE;
    // return;
    // }
    shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), LOWER_ALGAE_SETPOINT+shoulderOffset));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shoulder-encoder-val", shoulderMotor.getPosition().getValueAsDouble());
    SmartDashboard.putString("shoulderPosition", myVAR.toString());
    SmartDashboard.putNumber("shoulder torque", getTorque());
    SmartDashboard.putNumber("shold-abs-enc", absoluteEncoder.get());
    SmartDashboard.putNumber("shoulderOffset", shoulderOffset);
    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    shoulderMotor.set(speed);
  }

  private double getTorque() {
    return shoulderMotor.getTorqueCurrent().getValueAsDouble() * shoulderMotor.getMotorKT().getValueAsDouble();
  }

}