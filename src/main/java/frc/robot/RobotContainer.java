// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LED.LighNextLed;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
    LEDSubsystem m_led = new LEDSubsystem();
    Joystick joystick = new Joystick(0);

    public RobotContainer() {
        configureBindings(
            
        );
    }

    private void configureBindings() {
        new JoystickButton(joystick, 3)
        .toggleOnTrue(new InstantCommand(() -> m_led.setRGB(0, 0, 255))
        .andThen(new RepeatCommand(Commands.sequence(new InstantCommand(() -> m_led.DecreaseAllLedsBrightness()),
        new WaitCommand(0),
        new LighNextLed(m_led)))));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
