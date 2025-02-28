/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  private TalonFX leftMotor1 = new TalonFX(11, "iceberg-canivore");
  private TalonFX leftMotor2 = new TalonFX(31, "iceberg-canivore");
  private TalonFX rightMotor1 = new TalonFX(21, "iceberg-canivore");
  private TalonFX rightMotor2 = new TalonFX(41, "iceberg-canivore");

  //private Joystick joy1 = new Joystick(0);
     XboxController xboxController = new XboxController(0);
     XboxController fakexboxController = new XboxController(2);

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  @Override
  public void robotInit() {
    // m_robotContainer = new RobotContainer();
  }

  // @Override
  // public void robotPeriodic() {

  // }

  public void autonomousInit() {
  }

  public void disabledInit() {}
  final double kP = 0.05;

  @Override
  public void disabledPeriodic() {}
  double setpoint = 0;

  // @Override
  // public void autonomousInit() {
  //   m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  //   // schedule the autonomous command (example)
  //   if (m_autonomousCommand != null) {
  //     m_autonomousCommand.schedule();
  //   }
  // }

  public void testPeriodic() {
    // get joystick command
    if (xboxController.getStartButtonPressed()) {
      setpoint = 2;  //How many feet to run on button 1
      System.out.println("BUTTON 1");
    } else if (xboxController.getBackButtonPressed()) {
      System.out.println("BUTTON 2");
      setpoint = 0;
    }

    // System.out.println(xboxController.isConnected());
    // System.out.println("FAKE: " + fakexboxController.isConnected());
    // System.out.println(xboxController.getLeftX());
    // System.out.println(xboxController.getName());
    // System.out.println(xboxController.getXButtonPressed());
    // System.out.println(xboxController.getLeftTriggerAxis());
    // System.out.println(xboxController.getStartButtonPressed());

    // get sensor position
    double sensorPosition = encoder.get() * kDriveTick2Feet;

    // calculations
    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;

    // output to motors
    leftMotor1.set(outputSpeed);
    leftMotor2.set(outputSpeed);
    rightMotor1.set(-outputSpeed);
    rightMotor2.set(-outputSpeed);
  }

  // @Override
  // public void autonomousPeriodic() {}
  
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
    SmartDashboard.updateValues();
    SmartDashboard.updateValues();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}
  // public void teleopPeriodic() {
  // }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    encoder.reset();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void autonomousPeriodic() {}
  @Override
  public void simulationInit() {}
  // public void testPeriodic() {
  // }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}