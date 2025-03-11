// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSubsystem extends SubsystemBase {

    /**
     * Creates a new ElevatorSubsystem.
     */

    public enum elevatorPosition {
        START, L1, L2, L3, L4, CORAL_STATION, GROUND, UPPER_ALGAE, LOWER_ALGAE
    }

    // 60:1 GEAR RATIO
    elevatorPosition myVAR = elevatorPosition.START;

    TalonFX elevatorMotor;

    public DigitalInput elevatorLimitSwitch;

    public PIDController elevatorPidController = new PIDController(0.02, 0, 0.001);

    ShoulderSubsystem shoulderSubsystem;

    public ElevatorSubsystem(ShoulderSubsystem shoulderSubsystem) {
        this.shoulderSubsystem = shoulderSubsystem;
        this.elevatorLimitSwitch = new DigitalInput(0);
        this.elevatorMotor = new TalonFX(51);
        elevatorPidController.setTolerance(0.6, 0.005);
        zeroElevatorEncoder();
    }

    public void zeroElevatorEncoder() {
        elevatorMotor.setPosition(0);
    }

    public void set(double speed) {
        elevatorMotor.set(speed);
    }

    public void reset() {
        if (elevatorLimitSwitch.get()) {
            if (this.getElevatorEncoder() >= 3) {
                this.set(elevatorPidController.calculate(this.getElevatorEncoder(), 0));
            } else {
                this.set(0.1);
            }
        } else {
            this.zeroElevatorEncoder();
            this.set(0);
        }
    }

    public double getElevatorEncoder() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    /**
     * @return true if past extension
     */
    public boolean extensionChecker() {
        double maxExtension;
        double shoulderEncoder = ((shoulderSubsystem.getShoulderEncoder() + 141) * 360) / 1000;

        double elevatorEncoder = ((getElevatorEncoder() / 6.375) + 36);
        // multiply by length

        maxExtension = 40 / Math.cos(shoulderEncoder);

        SmartDashboard.putNumber("distance-from-bumper", elevatorEncoder - maxExtension);

        if (maxExtension < elevatorEncoder) {
            return true;
        } else {
            return false;
        }
    }

    public void moveElevatorL1() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 0) {
            elevatorOFF();
            myVAR = elevatorPosition.L1;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 0));
    }

    public void moveElevatorL2() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 43.71) {
            elevatorOFF();
            myVAR = elevatorPosition.L2;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 43.71));
    }

    public void moveElevatorL3() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 102.275) {
            elevatorOFF();
            myVAR = elevatorPosition.L3;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 102.275));
    }

    public void moveElevatorL4() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.L4;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveElevatorCoralStation() {
        // if (elevatorMotor.getPosition().getValueAsDouble() >= 30.272) {
        // elevatorOFF();
        // myVAR = elevatorPosition.CORAL_STATION;
        // return;
        // }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 30.272));
    }

    public void moveElevatorGround() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 20.115) {
            elevatorOFF();
            myVAR = elevatorPosition.GROUND;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveElevatorUpperAlgae() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.UPPER_ALGAE;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveElevatorLowerAlgae() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.LOWER_ALGAE;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void elevatorOUT() {
        elevatorMotor.set(ElevatorConstants.Elevator_MOTOR_SPEED);
        if (getElevatorLimit(shoulderSubsystem.getShoulderEncoder()) > this
                .getElevatorEncoder()) {

        } else {
            // elevatorMotor.set(0);
        }

        // if (!extensionChecker()) {
        // elevatorMotor.set(ElevatorConstants.Elevator_MOTOR_SPEED);
        // } else {
        // elevatorOFF();
        // }
    }

    /**
     * 
     * @param x The Shoulder Encoder base Value, NO GEAR RATIOS INVOLVED DO NOT DO
     *          THE MATH FOR US.
     * @return
     */
    public double getElevatorLimit(double x) {
        double elevatorLimit;
        elevatorLimit = (0.0000622462 * (Math.pow(x, 3))) + (0.02255 * Math.pow(x, 2)) + (2.91 * x) + (169);
        return elevatorLimit;
    }

    public void elevatorLimiter() {
        if (getElevatorEncoder() > 225) {
            elevatorIN();
        }
    }

    public void elevatorIN() {
        extensionChecker();
        if (!elevatorLimitSwitch.get()) {
            zeroElevatorEncoder();
            elevatorOFF();
        } else {
            elevatorMotor.set(-ElevatorConstants.Elevator_MOTOR_SPEED);
        }
    }

    public void elevatorOFF() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elev-encoder-val", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("elevatorPosition", myVAR.toString());
        SmartDashboard.putBoolean("Limit Switch Elebator", elevatorLimitSwitch.get());
        SmartDashboard.putNumber("inner-encoder", ((shoulderSubsystem.getShoulderEncoder() + 141) * 360) / 1000);
        // This method will be called once per scheduler run
    }
}