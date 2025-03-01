// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANrange CANrange;
  TalonFX intakeMotor;
  double recordedTime;

  public IntakeSubsystem() {
    // this.intakeMotor = new TalonFX(61);
    this.intakeMotor = new TalonFX(61);
    this.CANrange = new CANrange(62);

  }

  public void intakeMotorON() {
    intakeMotor.set(INTAKE_MOTOR_SPEED);
  }

  public void intakeMotorOFF() {
    intakeMotor.set(0);
  }

  public double getTime() {
    recordedTime = Timer.getTimestamp();
    return recordedTime;
  }

  
  public void intakeGamePiece() {
    SmartDashboard.putBoolean("intakeWorking", (CANrange.getDistance().getValueAsDouble() > DISTANCE_FROM_PIECE / 39.3701));
    if (CANrange.getDistance().getValueAsDouble() > (DISTANCE_FROM_PIECE/39.3701)*2) {// might be broken hasn't
                                                                                              // been tested
      intakeMotorON();
    } else {
      if (recordedTime > Timer.getTimestamp() - INTAKE_PULSE_INTERVAL) {
        intakeMotorOFF();
      } else {
        if (recordedTime > Timer.getTimestamp() - (INTAKE_PULSE_INTERVAL + INTAKE_PULSE_LENGTH)) {
          intakeMotorON();
        } else {
          getTime();
          intakeMotorOFF();
        }

      }
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("range", CANrange.getDistance().getValueAsDouble());
    // This method will be called once per scheduler run

  }
}
