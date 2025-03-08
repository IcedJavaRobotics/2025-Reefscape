/************************ PROJECT SYSID ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

//package frc.robot;
package frc.robot;

//import frc.robot.stuylib.input.Gamepad;
import frc.robot.stuylib.input.Gamepad;
//import frc.robot.stuylib.input.gamepads.AutoGamepad;
import frc.robot.stuylib.input.gamepads.AutoGamepad;

//import frc.robot.constants.Ports;
import frc.robot.constants.Ports;
//import frc.robot.subsystems.AbstractSysID;
import frc.robot.subsystems.AbstractSysID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    public final AbstractSysID sysid = AbstractSysID.getInstance();

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystem

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {}

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Quasistatic Forward", sysid.quasistaticForward());
        autonChooser.addOption("Quasistatic Reverse", sysid.quasistaticReverse());
        autonChooser.addOption("Dynamic Forward", sysid.dynamicForward());
        autonChooser.addOption("Dynamic Reverse", sysid.dynamicReverse());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
