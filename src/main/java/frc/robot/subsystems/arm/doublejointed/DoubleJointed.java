/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.arm.doublejointed;

import static frc.robot.constants.Settings.Arm.DoubleJointed.*;

import frc.robot.stuylib.control.angle.AngleController;
import frc.robot.stuylib.control.angle.feedback.AnglePIDController;
import frc.robot.stuylib.control.angle.feedforward.AngleArmFeedforward;
import frc.robot.stuylib.math.Angle;
import frc.robot.stuylib.network.SmartBoolean;

import frc.robot.constants.Ports;
import frc.robot.constants.Settings.Arm.DoubleJointed.JointOne;
import frc.robot.constants.Settings.Arm.DoubleJointed.JointTwo;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DoubleJointed extends SubsystemBase {

    private final SparkMax jointOne;
    private final RelativeEncoder jointOneEncoder;
    private final AngleController jointOneController;

    private final SparkMax jointTwo;
    private final RelativeEncoder jointTwoEncoder;
    private final AngleController jointTwoController;

    private double jointOneVoltage;
    private double jointTwoVoltage;

    private final SmartBoolean jointOneRoutine;
    private final SmartBoolean jointTwoRoutine;

    public DoubleJointed() {
        jointOne = new SparkMax(Ports.Arm.JOINT_ONE, MotorType.kBrushless);
        SparkBaseConfig jointOneConfig = new SparkMaxConfig();
        jointOneConfig.encoder.positionConversionFactor(JointOne.POSITION_CONVERSION);
        jointOneConfig.encoder.velocityConversionFactor(JointOne.VELOCITY_CONVERSION);
        jointOne.configure(jointOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        jointOneEncoder = jointOne.getEncoder();

        jointOneController =
                new AnglePIDController(JointOne.kP, JointOne.kI, JointOne.kD)
                        .add(new AngleArmFeedforward(JointOne.kG.get()));

        jointTwo = new SparkMax(Ports.Arm.JOINT_TWO, MotorType.kBrushless);
        SparkBaseConfig jointTwoConfig = new SparkMaxConfig();
        jointTwoConfig.encoder.positionConversionFactor(JointTwo.POSITION_CONVERSION);
        jointTwoConfig.encoder.velocityConversionFactor(JointTwo.VELOCITY_CONVERSION);
        jointOne.configure(jointTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        jointTwoEncoder = jointTwo.getEncoder();

        jointTwoController =
                new AnglePIDController(JointTwo.kP, JointTwo.kI, JointTwo.kD)
                        .add(new AngleArmFeedforward(JointTwo.kG.get()));

        jointOneVoltage = 0;
        jointTwoVoltage = 0;

        jointOneRoutine = new SmartBoolean("Double Jointed/Joint One Routine", false);
        jointTwoRoutine = new SmartBoolean("Double Jointed/Joint Two Routine", false);
    }

    public double getJointOneVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(jointOneEncoder.getVelocity());
    }

    public double getJointOnePosition() {
        return Units.rotationsToRadians(jointTwoEncoder.getPosition());
    }

    public double getJointOneVoltage() {
        return jointOneVoltage;
    }

    public void setJointOneVoltage(double voltage) {
        this.jointOneVoltage = voltage;
        jointOne.setVoltage(voltage);
    }

    public double getJointTwoVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(jointOneEncoder.getVelocity());
    }

    public double getJointTwoPosition() {
        return Units.rotationsToRadians(jointTwoEncoder.getPosition());
    }

    public double getJointTwoVoltage() {
        return jointTwoVoltage;
    }

    public void setJointTwoVoltage(double voltage) {
        this.jointTwoVoltage = voltage;
        jointTwo.setVoltage(voltage);
    }

    public void setMode(boolean jointOneRoutine, boolean jointTwoRoutine) {
        this.jointOneRoutine.set(jointOneRoutine);
        this.jointTwoRoutine.set(jointTwoRoutine);
    }

    @Override
    public void periodic() {
        if (jointOneRoutine.get()) {
            jointTwo.setVoltage(
                    jointTwoController.update(
                            Angle.kZero, Angle.fromRadians(getJointTwoPosition())));
        } else if (jointTwoRoutine.get()) {
            jointOne.setVoltage(
                    jointOneController.update(
                            Angle.k90deg, Angle.fromRadians(getJointOnePosition())));
        }
    }
}
