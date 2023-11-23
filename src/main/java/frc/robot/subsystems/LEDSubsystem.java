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

    public LEDSubsystem() {
        m_led.setLength(m_LedBuffer.getLength());

        m_led.setData(m_LedBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public AddressableLEDBuffer getLedBuffer() {
        return m_LedBuffer;
    }

    public void setAllLedsStaticColorMode(LEDSubsystem led, int r, int g, int b) {
        for (var i = 0; i < led.m_LedBuffer.getLength(); i++) {
            m_LedBuffer.setRGB(i, r, g, b);
        }

        m_led.setData(m_LedBuffer);
    }

    public void setSpecificLedStaticColorMode(
            LEDSubsystem led, int r, int g, int b, int specifiedLed) {
        m_LedBuffer.setRGB(specifiedLed, r, g, b);
        m_led.setData(m_LedBuffer);
    }

    public void setAllLedsRainbowMode(LEDSubsystem led, int r, int g, int b) {
        int m_rainbowFirstPixelHue = 0;
        for (var i = 0; i < led.m_LedBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LedBuffer.getLength()));
            m_LedBuffer.setHSV(i, r, g, b);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }

    public void IncreaseAllLedsBrightness(LEDSubsystem led, int r, int g, int b) {
        int currentR, currentG, currentB;
        currentR = r;
        currentG = g;
        currentB = b;
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

    public void DecreaseAllLedsBrightness(LEDSubsystem led, int r, int g, int b) {
        int currentR, currentG, currentB;
        currentR = r;
        currentG = g;
        currentB = b;
        for (var i = 0; i < m_LedBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for blue
            m_LedBuffer.setRGB(i, currentR, currentG, currentB);
        }

        // increase brightness
        currentR -= 20;
        currentG -= 20;
        currentB -= 20;
        // Check bounds
        currentR %= 255;
        currentG %= 255;
        currentB %= 255;

        m_led.setData(m_LedBuffer);
    }
}
