// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Networking extends SubsystemBase {
  /** Creates a new Networking. */
  public Networking() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private NetworkTable table;

  public void robotInit() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("AudioPlayer");
    inst.putValue("Hallo World");

  }
}
