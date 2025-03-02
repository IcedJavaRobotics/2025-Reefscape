// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    /**
     * Creates a new ElevatorSubsystem.
     */

    public enum elevatorPosition {
        START, L1, L2, L3, L4, CORAL_STATION, GROUND, UPPER_ALGAE, LOWER_ALGAE
    }

    elevatorPosition myVAR = elevatorPosition.START;

    TalonFX elevatorMotor;
    public PIDController elevatorPidController = new PIDController(0.02, 0, 0.001);

    public ElevatorSubsystem() {

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

    public double getElevatorEncoder() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    public void moveElevatorL1() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.L1;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveElevatorL2() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.L2;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveElevatorL3() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.L3;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
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
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
            elevatorOFF();
            myVAR = elevatorPosition.CORAL_STATION;
            return;
        }
        elevatorMotor.set(elevatorPidController.calculate(elevatorMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveElevatorGround() {
        if (elevatorMotor.getPosition().getValueAsDouble() >= 10) {
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
    }

    public void elevatorIN() {

        elevatorMotor.set(-ElevatorConstants.Elevator_MOTOR_SPEED);
    }

    public void elevatorOFF() {
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elev-encoder-val", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("elevatorPosition", myVAR.toString());
        // This method will be called once per scheduler run
    }
}