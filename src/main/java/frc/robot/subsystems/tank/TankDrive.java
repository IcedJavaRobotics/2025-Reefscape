package frc.robot.subsystems.tank;

import static frc.robot.constants.Settings.TankDrive.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {

    private final SparkMax frontLeft;
    private final SparkMax backLeft;
    private final SparkMax frontRight;
    private final SparkMax backRight;
    
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private double voltage;

    public TankDrive() {
        frontLeft = new SparkMax(Ports.TankDrive.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        SparkBaseConfig frontLeftConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        frontLeftConfig.encoder.positionConversionFactor(POSITION_CONVERSION);
        frontLeftConfig.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        frontLeft.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        backLeft = new SparkMax(Ports.TankDrive.BACK_LEFT_MOTOR, MotorType.kBrushless);
        SparkBaseConfig backLeftConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(frontLeft);
        backLeftConfig.encoder.positionConversionFactor(POSITION_CONVERSION);
        backLeftConfig.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        backLeft.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        frontRight = new SparkMax(Ports.TankDrive.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        SparkBaseConfig frontRightConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        frontRightConfig.encoder.positionConversionFactor(POSITION_CONVERSION);
        frontRightConfig.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        frontRight.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        backRight = new SparkMax(Ports.TankDrive.BACK_RIGHT_MOTOR, MotorType.kBrushless);
        SparkBaseConfig backRightConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(frontRight);
        backRightConfig.encoder.positionConversionFactor(POSITION_CONVERSION);
        backRightConfig.encoder.velocityConversionFactor(VELOCITY_CONVERSION);
        backRight.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = frontLeft.getEncoder();
        rightEncoder = frontRight.getEncoder();

        voltage = 0;
    }

    public double getVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    public double getPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        frontLeft.setVoltage(voltage);
        frontRight.setVoltage(voltage);
    }
}
