// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(LEDConstants.kLEDHeader);
    AddressableLEDBuffer m_LedBuffer = new AddressableLEDBuffer(LEDConstants.kLEDBuffer);
    private int currentR, currentG, currentB;
    public int currentIndex = 0;

    public LEDSubsystem() {
        m_led.setLength(m_LedBuffer.getLength());
        m_led.setData(m_LedBuffer);
        m_led.start();
    }

    public enum LEDState {
        STATIC_ON,
        BLINKING,
        ANIMATING,
        STATIC_OFF
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public AddressableLEDBuffer getLedBuffer() {
        return m_LedBuffer;
    }

    public void setAllLedsStaticColorMode() {
        for (var i = 0; i < m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i, currentR, currentG, currentB);
        }

        m_led.setData(m_LedBuffer);
    }

    public void setSpecificLedStaticColorMode(int specifiedLed) {
        m_LedBuffer.setRGB(specifiedLed, currentR, currentG, currentB);
        m_led.setData(m_LedBuffer);
    }

    public void setRGB(int r, int g, int b) {
        currentR = r;
        currentG = g;
        currentB = b;
    }

    //TODO: FIX THIS
    /*public void setAllLedsRainbowMode(LEDSubsystem led, int r, int g, int b) {
        int m_rainbowFirstPixelHue = 0;
        for (var i = 0; i < led.m_LedBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LedBuffer.getLength()));
            m_LedBuffer.setHSV(i, r, g, b);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }*/

    public void IncreaseAllLedsBrightness() {
        for (var i = 0; i < m_LedBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue
            m_LedBuffer.setRGB(i, currentR, currentG, currentB);
        }

        // increase brightness
        currentR += 5;
        currentG += 5;
        currentB += 5;
        // Check bounds
        currentR %= 255;
        currentG %= 255;
        currentB %= 255;

        m_led.setData(m_LedBuffer);
    }

    public void DecreaseAllLedsBrightness() {

        for (var i = 0; i < m_LedBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue
            m_LedBuffer.setRGB(i, currentR, currentG, currentB - ((Math.abs(currentIndex - i)) * 50));
        }

        // increase brightness
        // Check bounds
        currentR %= 255;
        currentG %= 255;
        currentB %= 255;

        currentR = currentR < 0 ? 0 : currentR;
        currentG = currentG < 0 ? 0 : currentG;
        currentB = currentB < 0 ? 0 : currentB;

        m_led.setData(m_LedBuffer);
    }
}
