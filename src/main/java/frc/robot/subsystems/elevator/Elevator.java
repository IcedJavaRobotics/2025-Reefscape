/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.elevator;

import static frc.robot.constants.Settings.Elevator.*;

import frc.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder;

    private double voltage;

    public Elevator() {
        motor1 = new SparkMax(Ports.Elevator.MOTOR, MotorType.kBrushless);
        SparkBaseConfig motor1Config = new SparkMaxConfig();
        motor1Config.encoder.positionConversionFactor(POSITION_CONVERSION);
        motor1Config.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor1.getEncoder();

        motor2 = new SparkMax(Ports.Elevator.MOTOR2, MotorType.kBrushless);
        SparkBaseConfig motor2Config = new SparkMaxConfig();
        motor2Config.encoder.positionConversionFactor(POSITION_CONVERSION);
        motor2Config.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        motor2Config.follow(motor1);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        voltage = 0;
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motor1.setVoltage(voltage);
    }
}
