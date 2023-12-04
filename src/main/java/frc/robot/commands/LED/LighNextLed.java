// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LighNextLed extends InstantCommand {
    private LEDSubsystem m_led;
    public boolean isReverse = false;

    public LighNextLed(LEDSubsystem led) {
        m_led = led;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // ? increases or decreases the index of the current index based on the direction
        if (isReverse == false) {
            m_led.currentIndex += 1;
        } else if (isReverse == true) {
            m_led.currentIndex -= 1;
        }

        // ? changes the direction of animation if reaches the end of the buffer
        if (m_led.currentIndex == m_led.getLedBuffer().getLength() - 1) {
            m_led.currentIndex = 0;
        } else if (m_led.currentIndex == 0) {
            isReverse = false;
        }

        // ? sets the led int the current index to the desired color output
        m_led.setSpecificLedStaticColorMode(m_led.currentIndex);
    }
}
