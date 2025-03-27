// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

import static frc.robot.Constants.LimelightConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

    // private Spark blinkin = new Spark(0); //Creates a blinkin as if it were a
    // spark.
    // Creates a new LimelightSubsystem.

    private double lockedApriltag = -1;
    private double lastSeenApriltag = -1;

    public LimelightSubsystem() {

        HttpCamera httpCamera = new HttpCamera("sauron", "http://10.68.94.11:5801/");
        CameraServer.getVideo(httpCamera);
        Shuffleboard.getTab("LiveWindow").add(httpCamera);

        // CameraServer.getInstance().addCamera(httpCamera); Shuffleboard.getTab("Tab")
        // .add(httpCamera);

    }

    /**
     * @return The ID of the apriltag it can see
     */
    public double getTid() {
        return NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("tid").getDouble(0);
    }

    /**
     * @return the distance from the center of the limelight view to the apriltag
     *         horizontally
     */
    public double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("tx").getDouble(0);
    }

    /**
     * @return the distance from the center of the limelight view to the apriltag
     *         vertically
     */
    public double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("ty").getDouble(0);
    }

    /**
     * 
     * @return the size of the apriltag seen as a square, which roughly translates
     *         to distance away from the apriltag
     */
    public double getTa() {
        return NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("ta").getDouble(0);
    }

    /**
     * 
     * @param mode 1.0 for on, 0.0 for off
     */
    public void setFlasher(double mode) {
        NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("ledMode").setDouble(mode);
    }

    /**
     * whether or not the limelight sees a apriltag
     * 
     * @return true if it sees an apriltag, false if it didnt detect a tag
     */
    public Boolean tagDetected() {

        if (NetworkTableInstance.getDefault().getTable("limelight-sauron").getEntry("tv").getDouble(0) == 1) {
            return true;
        }
        return false;
    }

    public void lockApriltag(){
        this.lockedApriltag = lastSeenApriltag;
    }

    /**
     * command for calculating limelight distance
     * 
     * @return horizontal floor distance from robot to apriltag
     */
    public double getDistance() {

        double targetOffsetAngle_Vertical = getTy();
        double goalHeightInches = LimelightConstants.APRILTAG_HEIGHT;
        double angleToGoalRadians = (LimelightConstants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (3.14159 / 180);

        if (getTid() == 4 || getTid() == 5) {
            goalHeightInches = LimelightConstants.APRILTAG_DOUBLE_SUBSTATION_HEIGHT;
        } // tid 4 and 5 are the double substations

        // calculate distance
        if (getTy() >= 0) {
            return (goalHeightInches - LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
        } else {
            return (LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
        }

    }

    /**
     * Based on the ID, returns the heading to turn to the reef to place
     * @return the robot goal heading
     */
    public double getReefHeading(){

        if(lockedApriltag == 21 || lockedApriltag == 7){
            return 0;
        } else if(lockedApriltag == 19 || lockedApriltag == 8){
            return 60;
        } else if(lockedApriltag == 20 || lockedApriltag == 9){
            return 120;
        } else if(lockedApriltag == 18 || lockedApriltag == 10){
            return 179.99;
        } else if(lockedApriltag == 22 || lockedApriltag == 11){
            return -120;
        } else if(lockedApriltag == 17 || lockedApriltag == 6){
            return -60;
        }
        // if(id==11){
        //     return 90;
        // }
         return 6894;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(getTid() != -1){
            lastSeenApriltag = this.getTid();
        }
        SmartDashboard.putNumber("apriltag", getTid());
        SmartDashboard.putNumber("Locked apriltag", lockedApriltag);
        SmartDashboard.putNumber("last apriltag", lastSeenApriltag);


    }
}