// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActuatorInCommand;
import frc.robot.commands.ActuatorOutCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CandleRed;
import frc.robot.commands.ElevatorINCommand;
import frc.robot.commands.ElevatorOUTCommand;
import frc.robot.commands.ExampleCommand;

import frc.robot.commands.WristCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.commands.cursorControls.CursorDownCommand;
import frc.robot.commands.cursorControls.CursorLeftCommand;
import frc.robot.commands.cursorControls.CursorRightCommand;
import frc.robot.commands.cursorControls.CursorUpCommand;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.ToggleAuxLockCommand;
//import frc.robot.commands.TestMotorCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SelectorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final CandleSubsystem candleSubsystem = new CandleSubsystem();

  private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final TestSubsystem testSubsystem = new TestSubsystem();
  private final ActuatorSubsystem actuatorSubsystem = new ActuatorSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

        private final SelectorSubsystem selectorSubsystem = new SelectorSubsystem();

   XboxController xboxController = new XboxController(0);
   XboxController auXboxController = new XboxController(1);

  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    candleSubsystem.setCandleJavaBlue();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }
    
  private double getRightX(){
    return -m_driverController.getRightX();
  }
  private double getLeftX(){
    return -m_driverController.getLeftX();
  }
  private double getLeftY(){
    return -m_driverController.getLeftY();
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                                .withControllerRotationAxis(() -> getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.9)//Can be changed to alter speed
                                                                .allianceRelativeControl(true);





  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> m_driverController.getRightX(),
                                                                                             () -> m_driverController.getRightY())
                                                                                             .headingWhile(true);


  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));


        new JoystickButton(xboxController, XboxController.Button.kB.value)
        .whileTrue(new CandleRed(candleSubsystem));


    // new JoystickButton(xboxController, XboxController.Button.kY.value)
    //     .whileTrue(new TestMotorCommand(testSubsystem));
        

    // new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
    //     .whileTrue(new IntakeCommand(intakeSubsystem));

    new JoystickButton(xboxController, XboxController.Button.kB.value)
    .whileTrue(new ZeroGyroCommand(drivebase));

    // new JoystickButton(xboxController, XboxController.Button.kA.value)
    //   .whileTrue(new ActuatorOutCommand(actuatorSubsystem));       

    // new JoystickButton(xboxController, XboxController.Button.kX.value)
    //   .whileTrue(new ActuatorInCommand(actuatorSubsystem));

    
    new JoystickButton(xboxController, XboxController.Button.kX.value)
      .whileTrue(new WristCommand(wristSubsystem));

      new JoystickButton(xboxController, XboxController.Button.kA.value)
        .whileTrue(new IntakeCommand(intakeSubsystem));

        new JoystickButton(xboxController, XboxController.Button.kY.value)
        .whileTrue(new IntakeOutCommand(intakeSubsystem));



    new JoystickButton(xboxController, XboxController.Button.kStart.value)
      .whileTrue(new ShoulderCommand(shoulderSubsystem, 1));

      new JoystickButton(xboxController, XboxController.Button.kBack.value)
      .whileTrue(new ShoulderCommand(shoulderSubsystem, -1));

      new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new ElevatorINCommand(elevatorSubsystem));

      new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new ElevatorOUTCommand(elevatorSubsystem));

        

      new POVButton(auXboxController, 180) /* D-Pad pressed DOWN */
                                .whileTrue(new CursorDownCommand(selectorSubsystem));

                new POVButton(auXboxController, 0) /* D-Pad pressed UP */
                                .whileTrue(new CursorUpCommand(selectorSubsystem));

                new POVButton(auXboxController, 90) /* D-Pad pressed Right */
                                .whileTrue(new CursorRightCommand(selectorSubsystem));

                new POVButton(auXboxController, 270) /* D-Pad pressed Left */
                                .whileTrue(new CursorLeftCommand(selectorSubsystem));

                                new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new ToggleAuxLockCommand(selectorSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
