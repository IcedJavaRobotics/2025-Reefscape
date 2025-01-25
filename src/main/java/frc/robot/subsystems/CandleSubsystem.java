// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CandleSubsystem extends SubsystemBase {
  CANdle candle;

  /** Creates a new CandleSubsystem. */
  public CandleSubsystem() {

    CANdle candle = new CANdle(Constants.CANDLE_ID); 
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    //set led type
    candle.configAllSettings(config);
    this.candle = candle;
  }
public void setCandleRed(){
  candle.setLEDs(255, 0, 0);
}

  public void setCandleGreen(){
    candle.setLEDs(0, 255, 0);
  }

  public void setCandleBlue(){
    candle.setLEDs(0, 0, 255);
  }
  public void setCandleJavaBlue(){
    candle.setLEDs(0, 247, 255);
  }
  public void setCandleFire(){
    FireAnimation fireAnim = new FireAnimation();
    candle.animate(fireAnim);
  }
public void setCandleRainbow(){
  RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
  candle.animate(rainbowAnim);
}
/**
 * <p> If the aliance color is blue turn the Candle blue. If the aliance color is red turn the Candle red. If there is no alliance color recieved turn the Candle green.
 */
public void autoCandle() {
    Optional<Alliance> color = DriverStation.getAlliance();
    if (color.isPresent()) {
      if (color.get() == Alliance.Red) {
        setCandleRed();

      }

      if (color.get() == Alliance.Blue) {
        setCandleBlue();
        
      }
    }
    else {
      setCandleGreen();
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
