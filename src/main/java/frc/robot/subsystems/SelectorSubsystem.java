// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.moveToCommands.MoveLeftL1Command;
import frc.robot.commands.moveToCommands.MoveLeftL2Command;
import frc.robot.commands.moveToCommands.MoveLeftL3Command;
import frc.robot.commands.moveToCommands.MoveLeftL4Command;
import frc.robot.commands.moveToCommands.MoveLowerAlgaeCommand;
import frc.robot.commands.moveToCommands.MoveRightL1Command;
import frc.robot.commands.moveToCommands.MoveRightL2Command;
import frc.robot.commands.moveToCommands.MoveRightL3Command;
import frc.robot.commands.moveToCommands.MoveRightL4Command;
import frc.robot.commands.moveToCommands.MoveUpperAlgaeCommand;

public class SelectorSubsystem extends SubsystemBase {

    ShoulderSubsystem shoulderSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    WristSubsystem wristSubsystem;

    /** Creates a new SelectorSubsystem. */
    public SelectorSubsystem(ShoulderSubsystem shoulderSubsystem, ElevatorSubsystem elevatorSubsystem,
            WristSubsystem wristSubsystem) {
        smartDashboardDisplay();
        this.shoulderSubsystem = shoulderSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    boolean auxLock = false; // false is unlocked, true is locked
    boolean driverLock = false; // false is unlocked, true is locked
    int[][] selectorArray = { { -1, 2, 5, -1 }, { 0, 3, 6, 8 }, { 1, 4, 7, 9 } };

    int[] cursor = { 1, 0 };

    boolean[] smartArray = { false, false, false, false, false, false, false, false, false, false };

    /**
     * @p True is locked
     */
    public void driverLockOn() {
        driverLock = true;
    }

    /**
     * @p False is unlocked
     */
    public void DriverLockOff() {
        driverLock = false;
    }

    /**
     * @p True is locked, false is unlocked
     */
    public void toggleAuxLock() {
        auxLock = !auxLock;
    }

    /**
     * @p False is unlocked
     */
    public void auxLockOff() {
        auxLock = false;
    }

    public void cursorUp() {
        if (auxLock) {
            return;
        }

        cursor[1]--;
        if (cursor[1] == -1) {
            cursor[1] += 4;
        }
        if (cursor[1] == 0 && cursor[0] == 0) {
            cursor[1] += 2;
        }
        smartDashboardDisplay();
    }

    public void cursorDown() {
        if (auxLock) {
            return;
        }
        cursor[1]++;
        if (cursor[1] == 4) {
            cursor[1] -= 4;
        }
        if (cursor[1] == 3 && cursor[0] == 0) {
            cursor[1] -= 2;
        }
        smartDashboardDisplay();
    }

    public void cursorRight() {
        if (auxLock) {
            return;
        }
        cursor[0]++;
        if (cursor[0] == 3 && (cursor[1] == 0 || cursor[1] == 3)) {
            cursor[0] -= 2;
        } else if (cursor[0] == 3) {
            cursor[0] -= 3;
        }
        smartDashboardDisplay();
    }

    public void cursorLeft() {
        if (auxLock) {
            return;
        }
        cursor[0]--;
        if (cursor[0] == 0 && (cursor[1] == 0 || cursor[1] == 3)) {
            cursor[0] += 2;
        } else if (cursor[0] == -1) {
            cursor[0] += 3;
        }
        smartDashboardDisplay();
    }

    /*
     * this sets all values execpt the values the cursor is on to false and the
     * cursor value to true
     */
    public void displaySetter() {
        for (int i = 0; i < smartArray.length; i++) {
            if (i == selectorArray[cursor[0]][cursor[1]]) {
                smartArray[i] = true;
            } else {
                smartArray[i] = false;
            }
        }
    }

    public void smartDashboardDisplay() {
        switch (selectorArray[cursor[0]][cursor[1]]) {
            case 0:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveLeftL4Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 1:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveRightL4Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 2:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveUpperAlgaeCommand(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 3:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveLeftL3Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 4:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveRightL3Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 5:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveLowerAlgaeCommand(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 6:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveLeftL2Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 7:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveRightL2Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 8:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveLeftL1Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
            case 9:
                displaySetter();
                if (auxLock && driverLock) {
                    CommandScheduler.getInstance()
                            .schedule(new MoveRightL1Command(this.shoulderSubsystem, this.elevatorSubsystem,
                                    this.wristSubsystem));
                }
                break;
        }
        SmartDashboard.putBoolean("Left L4", smartArray[0]);
        SmartDashboard.putBoolean("Right L4", smartArray[1]);
        SmartDashboard.putBoolean("Upper Algae", smartArray[2]);
        SmartDashboard.putBoolean("Left L3", smartArray[3]);
        SmartDashboard.putBoolean("Right L3", smartArray[4]);
        SmartDashboard.putBoolean("Lower Algae", smartArray[5]);
        SmartDashboard.putBoolean("Left L2", smartArray[6]);
        SmartDashboard.putBoolean("Right L2", smartArray[7]);
        SmartDashboard.putBoolean("Left L1", smartArray[8]);
        SmartDashboard.putBoolean("Right L1", smartArray[9]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("AuxLock", auxLock);
        SmartDashboard.putBoolean("DriverLock", driverLock);
        // This method will be called once per scheduler run
    }
}