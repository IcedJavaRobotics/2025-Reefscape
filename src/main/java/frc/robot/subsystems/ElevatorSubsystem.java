// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  TalonFX elevatorMotor;
  public PIDController elevatorPidController = new PIDController(0.02, 0, 0.001);

  public ElevatorSubsystem() {

    this.elevatorMotor = new TalonFX(51, "rio");
    elevatorPidController.setTolerance(0.6, 0.005);
    zeroEncoder();
  }

  public void zeroEncoder() {
    elevatorMotor.setPosition(0);
  }

  public double getEncoder() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public void moveElevatorL1() {
    if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
      elevatorOFF();
      return;
    }
    elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveElevatorL2() {
    if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
      elevatorOFF();
      return;
    }
    elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveElevatorL3() {
    if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
      elevatorOFF();
      return;
    }
    elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveElevatorL4() {
    if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
      elevatorOFF();
      return;
    }
    elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveElevatorCoralStation() {
    if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
      elevatorOFF();
      return;
    }
    elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
  }

  public void moveElevatorZero() {
    if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
      elevatorOFF();
      return;
    }
    elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
  }

  public void elevatorOUT() {

    elevatorMotor.set(ElevatorConstants.Elevator_MOTOR_SPEED);
  }

  public void elevatorIN() {

    elevatorMotor.set(-ElevatorConstants.Elevator_MOTOR_SPEED);
  }

  public void elevatorOFF() {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
