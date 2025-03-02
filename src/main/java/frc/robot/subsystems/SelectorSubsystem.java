// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SelectorSubsystem extends SubsystemBase {
    /** Creates a new SelectorSubsystem. */
    public SelectorSubsystem() {
        smartDashboardDisplay();
    }

    boolean auxLock = false; // false is unlocked, true is locked
    int[][] selectorArray = { { -1, 2, 5, -1 }, { 0, 3, 6, 8 }, { 1, 4

            , 7, 9 } };

    int[] cursor = { 1, 0 };

    boolean[] smartArray = { false, false, false, false, false, false, false, false, false, false };

    public void toggleAuxLock() {
        auxLock = !auxLock;
    }

    public void auxLockOff() {
        auxLock = false;
    }

    public void cursorUp() {
        System.out.println("Cursor Up");
        if (auxLock) {
            return;
        }
        System.out.println("Not Locked");

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

    // TODO: Commands need implemented
    public void smartDashboardDisplay() {
        switch (selectorArray[cursor[0]][cursor[1]]) {
            case 0:
                displaySetter();
                // Left L4
                break;
            case 1:
                displaySetter();
                // Right L4
                break;
            case 2:
                displaySetter();
                // Upper algae
                break;
            case 3:
                displaySetter();
                // Left L3
                break;
            case 4:
                displaySetter();
                // Right L3
                break;
            case 5:
                displaySetter();
                // Lower Algae
                break;
            case 6:
                displaySetter();
                // Left L2
                break;
            case 7:
                displaySetter();
                // Right L2
                break;
            case 8:
                displaySetter();
                // Left L1
                break;
            case 9:
                displaySetter();
                // Right L1
                break;
        }
        System.out.println(smartArray[3]);
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

    /** Grabs the hatch. */
    // public Command cursorDownCommand() {
    // // implicitly require `this`
    // return this.runOnce(() -> cursorDown());
    // }
    // public Command cursorUpCommand() {
    // // implicitly require `this`
    // return this.runOnce(() -> cursorUp());
    // }
    // public Command cursorRightCommand() {
    // // implicitly require `this`
    // return this.runOnce(() -> cursorRight());
    // }
    // public Command cursorLeftCommand() {
    // // implicitly require `this`
    // return this.runOnce(() -> cursorLeft());
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}