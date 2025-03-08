/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.flywheel;

import static frc.robot.constants.Settings.Flywheel.*;

import frc.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Flywheel extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private double voltage;

    public Flywheel() {
        motor = new SparkMax(Ports.Flywheel.MOTOR, MotorType.kBrushless);
        SparkBaseConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(POSITION_CONVERSION);
        config.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();

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
        motor.setVoltage(voltage);
    }
}
