// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Meter;
import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */



File directory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive swerveDrive;


  public SwerveSubsystem() {
  try {
    swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                    new Pose2d(new Translation2d(Meter.of(1),
                                                                                                 Meter.of(4)),
                                                                               Rotation2d.fromDegrees(0)));
  } catch (IOException e) {
    // TODO Auto-generated catch block
    e.printStackTrace();
  }
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOrientated(ChassisSpeeds velocity){
    return run( () -> {
      System.out.println("driving field oriented");
      SmartDashboard.putNumber("Vx", velocity.vxMetersPerSecond);
      swerveDrive.driveFieldOriented(velocity);
    });
  }
}
