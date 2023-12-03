// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LED.LighNextLed;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDColorState;

public class RobotContainer {
    LEDSubsystem m_led = new LEDSubsystem();
    Joystick joystick = new Joystick(0);

    private LEDColorState ledColorState() {
        return LEDColorState.WHITE;
    }


    public RobotContainer() {
        configureBindings(

        );
    }

    private void configureBindings() {

        //* weirdo ripple effect */
        new JoystickButton(joystick, 3)
        .toggleOnTrue(new InstantCommand(() -> m_led.setRGB(0, 0, 255))
        .andThen(new RepeatCommand(Commands.sequence(new InstantCommand(() -> m_led.DecreaseAllLedsBrightness()),
        new WaitCommand(0),
        new LighNextLed(m_led)))));

        //* blinking effect */
        new JoystickButton(joystick, 2)
            .toggleOnTrue(new SelectCommand(
                Map.ofEntries(
                Map.entry(LEDColorState.RED,Commands.repeatingSequence(m_led.AllLEDSBlinking(255, 0, 0, 0.3))),
                Map.entry(LEDColorState.GREEN,Commands.repeatingSequence(m_led.AllLEDSBlinking(0, 255, 0, 0.3))),
                Map.entry(LEDColorState.BLUE,Commands.repeatingSequence(m_led.AllLEDSBlinking(0, 0, 255, 0.3)))
                ), () -> ledColorState()));

        //TODO: add command selection for different colors

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
