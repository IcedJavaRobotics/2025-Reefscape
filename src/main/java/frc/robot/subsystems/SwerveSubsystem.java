// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriverConstants.MAX_SPEED,
          new Pose2d(new Translation2d(Units.feetToMeters(1),
              Units.feetToMeters(4)),
              Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

                        try{
                                config = RobotConfig.fromGUISettings();
                              } catch (Exception e) {
                                // Handle exception as needed
                                e.printStackTrace();
                              }
                          
                              // Configure AutoBuilder last
                              AutoBuilder.configure(
                                      this::getPose, // Robot pose supplier
                                      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                                      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                      (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                                      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                                              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                                              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                                      ),
                                        //new 
                                      config, // The robot configuration
                                      () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                          
                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                          return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                      },
                                      this // Reference to this subsystem to set requirements
                              );

                      

  }

  public void resetPose(Pose2d pose) {
    this.getSwerveDrive().resetOdometry(pose);
}
public Pose2d getPose(){
    return this.getSwerveDrive().getPose();
}

public ChassisSpeeds getRobotRelativeSpeeds(){
    return this.getSwerveDrive().getRobotVelocity();
}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule swerveModule : swerveDrive.getModules()) {
      SmartDashboard.putNumber(swerveModule.toString(), swerveModule.getAbsolutePosition());
    }
    SmartDashboard.putNumber("Gyro", this.getPose().getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void driveRobotOriented(ChassisSpeeds velocity){
    swerveDrive.drive(velocity);
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.drive(velocity.get());
    });
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
}