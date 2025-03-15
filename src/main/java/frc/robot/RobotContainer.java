// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.actuator.ActuatorInCommand;
import frc.robot.commands.actuator.ActuatorOutCommand;
import frc.robot.commands.candle.CandleRed;
import frc.robot.commands.cursor.CursorDownCommand;
import frc.robot.commands.cursor.CursorLeftCommand;
import frc.robot.commands.cursor.CursorRightCommand;
import frc.robot.commands.cursor.CursorUpCommand;
import frc.robot.commands.cursor.ToggleAuxLockCommand;
import frc.robot.commands.elevator.ElevatorINCommand;
import frc.robot.commands.elevator.ElevatorOUTCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeOutSlowCommand;
import frc.robot.commands.misc.ArmDemonstrationCommand;
import frc.robot.commands.misc.ZeroGyroCommand;
import frc.robot.commands.moveToCommands.MoveGroundCommand;
import frc.robot.commands.moveToCommands.MoveLowerAlgaeCommand;
import frc.robot.commands.moveToCommands.MoveRightL1Command;
import frc.robot.commands.moveToCommands.MoveRightL2Command;
import frc.robot.commands.moveToCommands.MoveRightL3Command;
import frc.robot.commands.moveToCommands.MoveRightL4Command;
import frc.robot.commands.primary.AutoIntakeCommand;
import frc.robot.commands.primary.AutoPlaceCommand;
import frc.robot.commands.primary.ResetMotorsCommand;
import frc.robot.commands.shoulder.ShoulderCommand;
import frc.robot.commands.wrist.WristCommand;
import frc.robot.commands.wrist.WristHorizontalCommand;
import frc.robot.commands.wrist.WristTestCommand;
import frc.robot.commands.wrist.WristVerticalCommand;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
//import frc.robot.commands.TestMotorCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SelectorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final CandleSubsystem candleSubsystem = new CandleSubsystem();

        private final ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
        private final WristSubsystem wristSubsystem = new WristSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final TestSubsystem testSubsystem = new TestSubsystem();
        private final ActuatorSubsystem actuatorSubsystem = new ActuatorSubsystem();
        private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(shoulderSubsystem);

        private final SelectorSubsystem selectorSubsystem = new SelectorSubsystem(shoulderSubsystem, elevatorSubsystem,
                        wristSubsystem);

        private final SendableChooser<Command> autoChooser;

        XboxController driverController = new XboxController(DriverConstants.MAIN_DRIVER_PORT);
        XboxController auxController = new XboxController(DriverConstants.AUX_DRIVER_PORT);
        private final Joystick driverStation = new Joystick(DriverConstants.DRIVER_STATION_PORT);

        private final SwerveSubsystem drivebase = new SwerveSubsystem();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                candleSubsystem.setCandleJavaBlue();
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

                // elevatorSubsystem.setDefaultCommand(new RunCommand(() ->
                // elevatorSubsystem.reset(), elevatorSubsystem));
                // resets to 0
                // shoulderSubsystem.setDefaultCommand(
                // new RunCommand(() -> shoulderSubsystem.reset(() -> elevatorInEnough()),
                // shoulderSubsystem));

                autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be
                // `Commands.none()`
                SmartDashboard.putData("AutoSelec", autoChooser);

        }

        PIDController headingController = new PIDController(0.02, 0, 0);

        private boolean elevatorInEnough() {
                if (elevatorSubsystem.getElevatorEncoder() <= 50) {
                        return true;
                }
                return false;
        }

        private double getDeadzone() {
                if (auxController.getRightX() >= 0.5 || auxController.getRightX() <= -0.5) {
                        return 0;
                } else if (getLeftDriverTriggerValue()) {
                        return 0;
                }
                return DriverConstants.DEADBAND;
        }

        private double getLeftX() {
                return -driverController.getLeftX();
        }

        private double getLeftY() {
                return -driverController.getLeftY();
        }

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverController.getLeftY() * 1,
                        () -> driverController.getLeftX() * 1)
                        .withControllerRotationAxis(() -> getRightX())
                        .deadband(getDeadzone())
                        .scaleTranslation(1)// Can be changed to alter speed
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(() -> driverController.getRightX(),
                                        () -> driverController.getRightY())
                        .headingWhile(true);

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
                // new Trigger(m_exampleSubsystem::exampleCondition)
                // .onTrue(new ExampleCommand(m_exampleSubsystem));

                // new Trigger(() -> getRightTriggerValue())
                // .onTrue(new TestCommand());

                // Primary Commands
                new Trigger(() -> getRightDriverTriggerValue()) // CORAL STATION COMMAND
                                .whileTrue(new AutoIntakeCommand(intakeSubsystem, shoulderSubsystem,
                                                elevatorSubsystem, wristSubsystem));

                new Trigger(() -> getLeftDriverTriggerValue()) // Place Coral On Reef
                                .whileTrue(new AutoPlaceCommand(intakeSubsystem, shoulderSubsystem, elevatorSubsystem));

                // Driver movement
                new JoystickButton(driverController, XboxController.Button.kB.value)
                                .whileTrue(new ZeroGyroCommand(drivebase));
                // Wrist Movement Manual
                new POVButton(driverController, 90)
                                .whileTrue(new WristTestCommand(wristSubsystem, 1));
                new POVButton(driverController, 270)
                                .whileTrue(new WristTestCommand(wristSubsystem, -1));
                // Wrist Movement PID
                new JoystickButton(driverController, XboxController.Button.kX.value)
                                .whileTrue(new WristVerticalCommand(wristSubsystem));
                new JoystickButton(driverController, XboxController.Button.kA.value)
                                .whileTrue(new WristHorizontalCommand(wristSubsystem));

                // Shoulder Movement
                new JoystickButton(driverController, XboxController.Button.kStart.value)
                                .whileTrue(new ShoulderCommand(shoulderSubsystem, 1));
                new JoystickButton(driverController, XboxController.Button.kBack.value)
                                .whileTrue(new ShoulderCommand(shoulderSubsystem, -1));

                // Elevator Movement
                new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new ElevatorINCommand(elevatorSubsystem));
                new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new ElevatorOUTCommand(elevatorSubsystem));

                // Intake Control
                new JoystickButton(driverController, XboxController.Button.kY.value)
                                .whileTrue(new IntakeOutCommand(intakeSubsystem, false));
                new POVButton(driverController, 0)
                                .whileTrue(new IntakeOutSlowCommand(intakeSubsystem));
                new POVButton(driverController, 180)
                                .whileTrue(new IntakeCommand(intakeSubsystem, false));

                // ---------AUX CONTROLS
                // --------------------------------------------------------------

                // Grid navigation
                new Trigger(() -> getRightAuxTriggerValue()) // FOR SELECTOR SUBSYSTEM
                                .whileTrue(new ResetMotorsCommand(intakeSubsystem, shoulderSubsystem, elevatorSubsystem,
                                                wristSubsystem));

                // new POVButton(auxController, 180) /* D-Pad pressed DOWN */
                //                 .whileTrue(new CursorDownCommand(selectorSubsystem));
                // new POVButton(auxController, 0) /* D-Pad pressed UP */
                //                 .whileTrue(new CursorUpCommand(selectorSubsystem));
                // new POVButton(auxController, 90) /* D-Pad pressed Right */
                //                 .whileTrue(new CursorRightCommand(selectorSubsystem));
                // new POVButton(auxController, 270) /* D-Pad pressed Left */
                //                 .whileTrue(new CursorLeftCommand(selectorSubsystem));
                new POVButton(auxController, 90) /* D-Pad pressed DOWN */
                                .whileTrue(new IntakeCommand(intakeSubsystem, true));
                new POVButton(auxController, 270) /* D-Pad pressed UP */
                                .whileTrue(new IntakeOutCommand(intakeSubsystem, true));
                // new POVButton(auxController, 90) /* D-Pad pressed Right */
                //                 .whileTrue(new CursorRightCommand(selectorSubsystem));
                // new POVButton(auxController, 270) /* D-Pad pressed Left */
                //                 .whileTrue(new CursorLeftCommand(selectorSubsystem));


                // Movement presets
                new JoystickButton(auxController, XboxController.Button.kY.value)
                                .whileTrue(new MoveRightL4Command(shoulderSubsystem, elevatorSubsystem,
                                                wristSubsystem));
                new JoystickButton(auxController, XboxController.Button.kX.value)
                                .whileTrue(new MoveRightL1Command(shoulderSubsystem, elevatorSubsystem,
                                                wristSubsystem));
                new JoystickButton(auxController, XboxController.Button.kA.value)
                                .whileTrue(new MoveRightL2Command(shoulderSubsystem, elevatorSubsystem,
                                                wristSubsystem));
                new JoystickButton(auxController, XboxController.Button.kB.value)
                                .whileTrue(new MoveRightL3Command(shoulderSubsystem, elevatorSubsystem,
                                                wristSubsystem));
                new POVButton(auxController, 180) /* D-Pad pressed DOWN */
                                .whileTrue(new MoveLowerAlgaeCommand(shoulderSubsystem, elevatorSubsystem,
                                                wristSubsystem, intakeSubsystem));

                // new JoystickButton(auxController, XboxController.Button.)

                // Wrist PIDs
                new JoystickButton(auxController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new WristVerticalCommand(wristSubsystem));
                new JoystickButton(auxController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new WristHorizontalCommand(wristSubsystem));

                // Climber Controls
                new JoystickButton(auxController, XboxController.Button.kStart.value)
                                .whileTrue(new ActuatorInCommand(actuatorSubsystem));
                new JoystickButton(auxController, XboxController.Button.kBack.value)
                                .whileTrue(new ActuatorOutCommand(actuatorSubsystem));

                /*
                 * OTHER CONTROLS::
                 * DRIVER:
                 * -- LEFT JOYSTICK: TRANSLATION
                 * -- RIGHT JOYSTICK: ROTATION
                 * AUX:
                 * -- RIGHT JOYSTICK: ROBOT FACES LEFT CORAL STATION, AND VICE VERSA
                 * 
                 */

        }

        private boolean auxRightstickLeft() {
                if (auxController != null) {
                        if (auxController.getRightX() <= -0.5) {
                                return true;
                        }
                        return false;
                }
                return false;
        }

        private boolean getRightDriverTriggerValue() {
                if (driverController != null) {
                        if (driverController.getRightTriggerAxis() >= 0.5) {
                                return true;
                        }
                        return false;
                } else {
                        return false;
                }
        }

        private boolean getLeftDriverTriggerValue() {
                if (driverController != null) {
                        if (driverController.getLeftTriggerAxis() >= 0.5) {
                                return true;
                        }
                        return false;
                } else {
                        return false;
                }
        }

        private boolean getRightAuxTriggerValue() {
                if (auxController != null) {
                        if (auxController.getRightTriggerAxis() >= 0.5) {
                                return true;
                        }
                        return false;
                } else {
                        return false;
                }
        }

        private boolean getLeftTriggerValue() {
                if (driverController != null) {
                        if (driverController.getLeftTriggerAxis() >= 0.5) {
                                return true;
                        }
                        return false;
                } else {
                        return false;
                }
        }

        /**
         * 
         * @return True if manual, false if automatic
         */
        private boolean getManualSwitch() {
                return driverStation.getRawButtonPressed(7);
        }

        private double getRightX() {
                if (auxController.getRightX() >= 0.5) {
                        return headingController.calculate(
                                        drivebase.getSwerveDrive().getPose().getRotation().getDegrees(), 120);
                } else if (auxController.getRightX() <= -0.5) {
                        return headingController.calculate(
                                        drivebase.getSwerveDrive().getPose().getRotation().getDegrees(), -150);
                } else if (getLeftDriverTriggerValue()) { // aux movement gets priority over the auto align
                        double id = limelightSubsystem.getTid();
                        // shoulderSubsystem.coralStationPID();

                        if (id == 10 || id == 21) {
                                return headingController
                                                .calculate(drivebase.getSwerveDrive().getPose().getRotation()
                                                                .getDegrees(), 0);
                        }
                        if (id == 9 || id == 22) {
                                return headingController
                                                .calculate(drivebase.getSwerveDrive().getPose().getRotation()
                                                                .getDegrees(), 60);
                        }
                        if (id == 8 || id == 17) {
                                return headingController
                                                .calculate(drivebase.getSwerveDrive().getPose().getRotation()
                                                                .getDegrees(), 120);
                        }
                        if (id == 7 || id == 18) {
                                return headingController
                                                .calculate(drivebase.getSwerveDrive().getPose().getRotation()
                                                                .getDegrees(), 180);
                        }
                        if (id == 6 || id == 19) {
                                return headingController
                                                .calculate(drivebase.getSwerveDrive().getPose().getRotation()
                                                                .getDegrees(), 240);
                        }
                        if (id == 11 || id == 20) {
                                return headingController
                                                .calculate(drivebase.getSwerveDrive().getPose().getRotation()
                                                                .getDegrees(), 300);
                        }
                }
                return -driverController.getRightX();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // return null;
        }
}
