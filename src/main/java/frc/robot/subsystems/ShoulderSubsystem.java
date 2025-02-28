// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShoulderConstants.SHOULDER_MOTOR_SPEED;

public class ShoulderSubsystem extends SubsystemBase {

    string shoulderPosition = "start";

    TalonFX shoulderMotor;
    public PIDController shoulderPidController = new PIDController(0.02, 0, 0.001);

    /**
     * Creates a new ShoulderSubsystem.
     */
    public ShoulderSubsystem() {
        shoulderMotor = new TalonFX(50, "rio");
        shoulderPidController.setTolerance(0.6, 0.005);
        zeroShoulderEncoder();
    }

    public void shoulderMotorFRW() {
        shoulderMotor.set(SHOULDER_MOTOR_SPEED);
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

    public void moveShoulderL1() {
        /* This moves the shoulder to the position of L1 on the Reef and updates the string shoulderPosition */
        if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
            shoulderMotorOFF();
            shoulderPosition = "L1";
            return;
        }
        shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveShoulderL2() {
        /* This moves the shoulder to the position of L2 on the Reef and updates the string shoulderPosition */
        if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
            shoulderMotorOFF();
            shoulderPosition = "L2";
            return;
        }
        shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveShoulderL3() {
        /* This moves the shoulder to the position of L3 on the Reef and updates the string shoulderPosition */
        if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
            shoulderMotorOFF();
            shoulderPosition = "L3";
            return;
        }
        shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveShoulderL4() {
        /* This moves the shoulder to the position of L4 on the Reef and updates the string shoulderPosition*/
        if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
            shoulderMotorOFF();
            shoulderPosition = "L4";
            return;
        }
        shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveShoulderCoralStation() {
        /* This moves the shoulder to the position of the Coral Station to intake a peice and updates the string shoulderPosition*/
        if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
            shoulderMotorOFF();
            shoulderPosition = "Coral Station";
            return;
        }
        shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
    }

    public void moveShoulderGround() {
        /* This moves the shoulder to the lowest possible position and updates the string shoulderPosition*/
        if (shoulderMotor.getPosition().getValueAsDouble() >= 10) {
            shoulderMotorOFF();
            shoulderPosition = "Ground";
            return;
        }
        shoulderMotor.set(shoulderPidController.calculate(shoulderMotor.getPosition().getValueAsDouble(), 10));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder-val", shoulderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("shoulderPosition", shoulderPosition);
        // This method will be called once per scheduler run
    }

}
