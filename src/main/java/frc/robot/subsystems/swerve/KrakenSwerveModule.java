/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.swerve;

import static frc.robot.constants.Settings.Swerve.*;

import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.stuylib.control.Controller;
import frc.robot.stuylib.control.angle.AngleController;
import frc.robot.stuylib.control.angle.feedback.AnglePIDController;
import frc.robot.stuylib.control.feedback.PIDController;
import frc.robot.stuylib.control.feedforward.MotorFeedforward;
import frc.robot.stuylib.math.Angle;
import frc.robot.stuylib.network.SmartBoolean;
import frc.robot.stuylib.streams.angles.filters.ARateLimit;

import frc.robot.constants.Settings.Swerve.Drive;
import frc.robot.constants.Settings.Swerve.Encoder;
import frc.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenSwerveModule extends SubsystemBase {

    private final String id;
    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final Controller driveController;

    private final TalonFX turnMotor;
    private final CANcoder turnAbsoluteEncoder;
    private final AngleController turnController;

    private final SmartBoolean driveSysID;
    private final SmartBoolean turnSysID;

    private double driveVoltage;
    private double turnVoltage;

    public KrakenSwerveModule(String id, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        this.id = id;
        this.angleOffset = angleOffset;

        driveMotor = new TalonFX(driveID, CANBUS);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        driveMotor.getConfigurator().apply(driveConfig);

        driveController =
                new PIDController(Drive.kP, Drive.kI, Drive.kD)
                        .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        turnMotor = new TalonFX(turnID, CANBUS);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        driveMotor.getConfigurator().apply(turnConfig);

        turnAbsoluteEncoder = new CANcoder(encoderID);
        turnController =
                new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
                        .setSetpointFilter(new ARateLimit(MAX_MODULE_TURN))
                        .setOutputFilter(x -> -x);

        driveSysID = new SmartBoolean("Swerve/Modules/Config/Drive SysID Enabled", false);
        turnSysID = new SmartBoolean("Swerve/Modules/Config/Turn SysID Enabled", false);

        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    /************************************************/

    public String getID() {
        return id;
    }

    public double getDriveVoltage() {
        return driveVoltage;
    }

    public double getTurnVoltage() {
        return turnVoltage;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * Encoder.Drive.VELOCITY_CONVERSION;
    }

    public double getTurnVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(turnMotor.getVelocity().getValueAsDouble() * Encoder.Turn.VELOCITY_CONVERSION * 60);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble() * Encoder.Turn.POSITION_CONVERSION);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble() * Encoder.Drive.POSITION_CONVERSION, getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /************************************************/

    public void setDriveVoltage(double voltage) {
        driveVoltage = voltage;
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage) {
        turnVoltage = voltage;
        turnMotor.setVoltage(voltage);
    }

    public void setMode(boolean driveSysID, boolean turnSysID) {
        this.driveSysID.set(driveSysID);
        this.turnSysID.set(turnSysID);
    }

    /************************************************/

    @Override
    public void periodic() {

        if (DriverStation.isAutonomous()) {

            if (driveSysID.get()) {
                setTurnVoltage(
                        turnController.update(Angle.kZero, Angle.fromRotation2d(getAbsoluteAngle())));
            } else if (turnSysID.get()) {
                setDriveVoltage(driveController.update(0, getDriveVelocity()));
            }

        } else {
            setTurnVoltage(turnController.update(Angle.kZero, Angle.fromRotation2d(getAbsoluteAngle())));
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Drive Voltage", getDriveVoltage());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Turn Voltage", turnController.getOutput());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Angle", getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + id + "/Speed", getDriveVelocity());
        SmartDashboard.putNumber(
                "Swerve/Modules/" + id + "/Raw Encoder Angle",
                Units.rotationsToDegrees(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}
