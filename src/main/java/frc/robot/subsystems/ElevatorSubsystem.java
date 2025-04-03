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
import static frc.robot.Constants.ElevatorConstants.*;

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

    public double elevatorOffset;

    public ElevatorSubsystem(ShoulderSubsystem shoulderSubsystem) {
        this.shoulderSubsystem = shoulderSubsystem;
        this.elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMITSWITCH_CHANNEL);
        this.elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
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
        this.set(elevatorPidController.calculate(this.getElevatorEncoder(), SAFE_RESET_SETPOINT));
    }

    public double getElevatorEncoder() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }


    public void setOffset(double newOffset){
        this.elevatorOffset = newOffset;
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
        myVAR = elevatorPosition.L1;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (L1_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorL2() {
        myVAR = elevatorPosition.L2;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (L2_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorL3() {
        myVAR = elevatorPosition.L3;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (L3_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorL4() {
        myVAR = elevatorPosition.L4;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (L4_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorCoralStation() {
        myVAR = elevatorPosition.CORAL_STATION;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (CORAL_STATION_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorGround() {
        myVAR = elevatorPosition.GROUND;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (GROUND_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorUpperAlgae() {
        myVAR = elevatorPosition.UPPER_ALGAE;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (UPPER_ALGAE_SETPOINT + elevatorOffset)));
    }

    public void moveElevatorLowerAlgae() {
            myVAR = elevatorPosition.LOWER_ALGAE;
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (LOWER_ALGAE_SETPOINT + elevatorOffset)));
    }
    public void moveElevatorGroundVertical(){
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), (LOWER_ALGAE_SETPOINT + elevatorOffset)));
    }

    public void elevatorOUT() {
        if (getElevatorEncoder() >= ELEVATOR_LIMIT){
            elevatorMotor.set(0);
            return;
        }
        elevatorMotor.set(ELEVATOR_MOTOR_SPEED);
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
            elevatorMotor.set(-ELEVATOR_MOTOR_SPEED);
        }
    }

    public void elevatorOFF() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elev-encoder-val", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("elevatorPosition", myVAR.toString());
        SmartDashboard.putBoolean("Limit Switch Elebator", !elevatorLimitSwitch.get());
        SmartDashboard.putNumber("inner-encoder", ((shoulderSubsystem.getShoulderEncoder() + 141) * 360) / 1000);
        // This method will be called once per scheduler run
    }
}