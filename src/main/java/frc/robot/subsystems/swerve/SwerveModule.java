/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package frc.robot.subsystems.swerve;

import static frc.robot.constants.Settings.Arm.SingleJointed.POSITION_CONVERSION;
import static frc.robot.constants.Settings.Arm.SingleJointed.VELOCITY_CONVERSION;
import static frc.robot.constants.Settings.Swerve.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SwerveModule extends SubsystemBase {

    private final String id;
    private final Rotation2d angleOffset;

    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final Controller driveController;

    private final SparkMax turnMotor;
    private final RelativeEncoder turnEncoder;
    private final CANcoder turnAbsoluteEncoder;
    private final AngleController turnController;

    private final SmartBoolean driveSysID;
    private final SmartBoolean turnSysID;

    private double driveVoltage;
    private double turnVoltage;

    public SwerveModule(String id, Rotation2d angleOffset, int turnID, int driveID, int encoderID) {
        this.id = id;
        this.angleOffset = angleOffset;

        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        SparkBaseConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveConfig.encoder.velocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        driveController =
                new PIDController(Drive.kP, Drive.kI, Drive.kD)
                        .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        SparkBaseConfig turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.encoder.positionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnConfig.encoder.velocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnEncoder = turnMotor.getEncoder();
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
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(turnEncoder.getVelocity() * 60);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
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
