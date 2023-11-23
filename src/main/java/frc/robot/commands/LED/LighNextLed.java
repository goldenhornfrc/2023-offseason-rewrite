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
  private int R,G,B;
  public int currentIndex = 0;

  public LighNextLed(LEDSubsystem ledSubsystem, int r, int g, int b) {
    m_led = ledSubsystem;
    R = r;
    G = g;
    B = b;
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.setSpecificLedStaticColorMode(m_led, R, G, B, currentIndex);
    currentIndex += 1;
  }
}
