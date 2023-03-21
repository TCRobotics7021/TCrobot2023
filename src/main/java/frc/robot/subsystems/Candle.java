// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase {
  /** Creates a new Candle. */
  private final CANdle candle = new CANdle(15);
  private int LedCount = 300;
  private Animation mode0 = null;
  private Animation mode1 = new FireAnimation(.5, .7, LedCount, .7, .5);
  private Animation mode2 = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
  private Animation mode3 = new StrobeAnimation(240, 10, 180, 0, 98.0/256.0,LedCount);
  private Animation mode4 = new RgbFadeAnimation(.7,98.0/256.0, LedCount);
  private Animation mode5 = new RainbowAnimation(.5, 1, LedCount);
  private Animation mode6 = new StrobeAnimation(255, 255, 0, 0, 98.0/256.0, LedCount);


  public Candle() {
    CANdleConfiguration config= new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);

  }
  public void setMode(int mode){
    switch(mode){
      case 0: candle.animate(mode0); break;
      case 1: candle.animate(mode1); break;
      case 2: candle.animate(mode2); break;
      case 3: candle.animate(mode3); break;
      case 4: candle.animate(mode4); break;
      case 5: candle.animate(mode5); break;
      case 6: candle.animate(mode6); break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
