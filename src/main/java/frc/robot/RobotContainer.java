// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Intake.IntakeStart;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.LED.LighNextLed;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDColorState;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.Map;

public class RobotContainer {
    public final LEDSubsystem m_led = new LEDSubsystem();
    public final Swerve m_swerve = new Swerve();
    public final ArmSubsystem m_arm = new ArmSubsystem();
    public final LimelightSubsystem m_lime = new LimelightSubsystem();
    public final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick thirdController = new Joystick(2);

    private LEDColorState ledColorState() {
        return LEDColorState.WHITE;
    }

    public RobotContainer() {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        m_swerve.setFieldOriented();
        m_swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_swerve,
                        m_lime,
                        () -> -driver.getRawAxis(1),
                        () -> -driver.getRawAxis(0),
                        () -> -driver.getRawAxis(4),
                        m_swerve::getIsFieldOriented));
        configureBindings();
    }

    private void configureBindings() {

        // * weirdo ripple effect */
        new JoystickButton(thirdController, 1)
                .toggleOnTrue(
                        new InstantCommand(() -> m_led.setRGB(0, 0, 255))
                                .andThen(
                                        new RepeatCommand(
                                                Commands.sequence(
                                                        new InstantCommand(() -> m_led.decreaseAllLedsBrightness()),
                                                        new WaitCommand(0),
                                                        new LighNextLed(m_led)))));

        // * blinking effect */
        new JoystickButton(thirdController, 3)
                .toggleOnTrue(
                        new SelectCommand(
                                Map.ofEntries(
                                        Map.entry(
                                                LEDColorState.WHITE,
                                                Commands.repeatingSequence(m_led.setAllLedsBlinking(255, 255, 255, 0.3))),
                                        Map.entry(
                                                LEDColorState.RED,
                                                Commands.repeatingSequence(m_led.setAllLedsBlinking(255, 0, 0, 0.3))),
                                        Map.entry(
                                                LEDColorState.GREEN,
                                                Commands.repeatingSequence(m_led.setAllLedsBlinking(0, 255, 0, 0.3))),
                                        Map.entry(
                                                LEDColorState.BLUE,
                                                Commands.repeatingSequence(m_led.setAllLedsBlinking(0, 0, 255, 0.3)))),
                                () -> ledColorState()));

        Trigger intakeFull = new Trigger(m_intake::getIntakeHasObject);

        // * flush orange color RGB value = 0,255,127 */
        // ? while angle lock is active, leds will blink in flush orange color with 0.1s blinks
        new JoystickButton(driver, 3)
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setWantsHeadingLock(true);
                                }))
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setHeadingLockTargetAngle(90);
                                }))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(0, 255, 127, 0.1)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()));

        new JoystickButton(driver, 2)
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setWantsHeadingLock(true);
                                }))
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setHeadingLockTargetAngle(180);
                                }))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(0, 255, 127, 0.1)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()));

        new JoystickButton(driver, 1)
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setWantsHeadingLock(true);
                                }))
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setHeadingLockTargetAngle(270);
                                }))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(0, 255, 127, 0.1)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()));

        new JoystickButton(driver, 4)
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setWantsHeadingLock(true);
                                }))
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    m_swerve.setHeadingLockTargetAngle(0);
                                }))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(0, 255, 127, 0.1)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()));

        // ! due to limited amount of buttons to bind, some arm angle commands should be done by
        // operator
        // ? led will be static color when lifting object to specific height to reduce distraction
        // * arm angle for cone actions */
        new JoystickButton(driver, 7)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmConeLow)))
                .onTrue(new InstantCommand(() -> m_led.setRGB(255, 255, 0)))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        new JoystickButton(driver, 5)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmConeMid)))
                .onTrue(new InstantCommand(() -> m_led.setRGB(255, 255, 0)))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        // ! because of lack of buttons, intake control and human player is binded to operator
        // ? blinking led animation is used when picking up cone from ground and from human player
        new JoystickButton(operator, 4)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmConeIntake)))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(255, 255, 0, 0.2)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        new JoystickButton(operator, 9)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmConeHuman)))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(255, 255, 0, 0.2)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        // !there is high setting because of the new catapult feature which enables us to score high
        // cube
        // ? led will be static color when lifting object to specific height to reduce distraction
        // * arm angle for cube actions */
        new JoystickButton(driver, 8)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmCubeLow)))
                .onTrue(new InstantCommand(() -> m_led.setRGB(0, 0, 255)))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        new JoystickButton(driver, 6)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmCubeMid)))
                .onTrue(new InstantCommand(() -> m_led.setRGB(0, 0, 255)))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        new JoystickButton(operator, 3) // ! i ran out of buttons to bind in driver controller
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmCubeHigh)))
                .onTrue(new InstantCommand(() -> m_led.setRGB(0, 0, 255)))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        // ! because of lack of buttons, intake control and human player is binded to operator
        new JoystickButton(operator, 1)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmCubeIntake)))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(0, 0, 255, 0.3)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        new JoystickButton(operator, 10)
                .toggleOnTrue(new InstantCommand(() -> m_arm.setArmAngle(ArmConstants.kArmCubeHuman)))
                .whileTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(0, 0, 255, 0.3)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new InstantCommand(() -> m_arm.stopArm()));

        // * arm home command to set arm angle to 0 degrees */
        new JoystickButton(operator, 8).onTrue(new InstantCommand(() -> m_arm.resetArm(0)));

        // ! intake is most vulnurable when enabled
        // ? leds will be rose color to indicate that intake is active
        // * intake controls are done by operator, having to do this would be too much for driver
        new JoystickButton(operator, 5)
                .toggleOnTrue(new IntakeStart(m_intake, 0.5))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(255, 0, 127, 0.2)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new IntakeStop(m_intake));

        new JoystickButton(operator, 6)
                .toggleOnTrue(new IntakeStart(m_intake, -0.5))
                .onTrue(new InstantCommand(() -> m_led.setAllLedsBlinking(255, 0, 127, 0.2)))
                .onFalse(new InstantCommand(() -> m_led.setRGB(0, 0, 0)))
                .onFalse(new InstantCommand(() -> m_led.setAllLedsStaticColorMode()))
                .onFalse(new IntakeStop(m_intake));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
