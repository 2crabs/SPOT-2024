// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kLED;

public class LEDSubsystem extends SubsystemBase {
  private boolean idle;

  private AddressableLED leds;
  private AddressableLEDBuffer ledBuffer;

  private int patternRainbowStart = 0;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    leds = null;
    idle = true;
    ledBuffer = new AddressableLEDBuffer(kLED.STRIP_LENGTH);
  }

  public void setLEDObject(AddressableLED ledObj) {
    leds = ledObj;
  }

  @Override
  public void periodic() {
    if(leds != null && idle) {
      rainbowPattern();
    }
  }

  public void idleMode(double rot) {
    idle = true;
    patternRainbowStart = (int)Math.round(rot/2);
  }

  /** 
   * Sets the color of the LED strip
   * @param color the color that the LEDs will be set to in RGB.
   */
  public void setColorRGB(int[] color) {
    for(int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    leds.setData(ledBuffer);

    idle = false;
  }

  /** 
   * Sets the color of the LED strip
   * @param color the color that the LEDs will be set to in HSV.
   */
  public void setColorHSV(int[] color) {
    for(int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, color[0], color[1], color[2]);
    }

    leds.setData(ledBuffer);

    idle = false;
  }

  /** ok i pull up */
  public void rainbowPattern() {
    for(int i = 0; i < ledBuffer.getLength(); i++) {
      int hue = (patternRainbowStart + (i*5 /* / ledBuffer.getLength()*/)) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }

    leds.setData(ledBuffer);
  }
}
