// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class AllLedsStaticColor extends CommandBase {
    private LEDSubsystem m_led;

    /** Creates a new AllLedsStaticColor. */
    public AllLedsStaticColor(LEDSubsystem ledSubsystem) {
        m_led = ledSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_led.setAllLedsStaticColorMode();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
