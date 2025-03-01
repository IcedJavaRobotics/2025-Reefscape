// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CandleRed;
import frc.robot.commands.ElevatorINCommand;
import frc.robot.commands.ElevatorOUTCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveCoralStationCommand;
import frc.robot.commands.MoveGroundCommand;
import frc.robot.commands.MoveL1Command;
import frc.robot.commands.MoveL2Command;
import frc.robot.commands.MoveL3Command;
import frc.robot.commands.MoveL4Command;
import frc.robot.commands.TestMotorCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
        private final CandleSubsystem candleSubsystem = new CandleSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
        private final WristSubsystem wristSubsystem = new WristSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final TestSubsystem testSubsystem = new TestSubsystem();

        XboxController xboxController = new XboxController(0);
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                candleSubsystem.setCandleJavaBlue();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
         * with an arbitrary predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * null null null null null null null null null null null null {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or null null null null null null null null null null
         * null null {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
                // new Trigger(m_exampleSubsystem::exampleCondition)
                // .onTrue(new ExampleCommand(m_exampleSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kB.value)
                                .whileTrue(new CandleRed(candleSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kA.value)
                                .whileTrue(new WristCommand(wristSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kY.value)
                                .whileTrue(new TestMotorCommand(testSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new IntakeCommand(intakeSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kA.value)
                                .whileTrue(new ElevatorINCommand(elevatorSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kX.value)
                                .whileTrue(new ElevatorOUTCommand(elevatorSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kX.value)
                                .whileTrue(new MoveL1Command(shoulderSubsystem, elevatorSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kY.value)
                                .whileTrue(new MoveL2Command(shoulderSubsystem, elevatorSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kB.value)
                                .whileTrue(new MoveL3Command(shoulderSubsystem, elevatorSubsystem));

                new JoystickButton(xboxController, XboxController.Button.kA.value)
                                .whileTrue(new MoveL4Command(shoulderSubsystem, elevatorSubsystem));

                new POVButton(xboxController, 0) /* D-Pad pressed UP */
                                .whileTrue(new MoveCoralStationCommand(shoulderSubsystem, elevatorSubsystem));

                new POVButton(xboxController, 180) /* D-Pad pressed DOWN */
                                .whileTrue(new MoveGroundCommand(shoulderSubsystem, elevatorSubsystem));

                // Schedule `exampleMethodCommand` when the Xbox controller's B button is
                // pressed,
                // cancelling on release.
                m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return Autos.exampleAuto(m_exampleSubsystem);
        }
}
