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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.SetArmClosedLoop;
import frc.robot.commands.Arm.SetArmOpenLoop;
import frc.robot.commands.Intake.IntakeStart;
import frc.robot.commands.LED.LighNextLed;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDColorState;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class RobotContainer {
    public final LEDSubsystem m_led = new LEDSubsystem();
    public final Swerve m_swerve = new Swerve();
    public final ArmSubsystem m_arm = new ArmSubsystem();
    public final LimelightSubsystem m_lime = new LimelightSubsystem();
    public final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick thirdJoystick = new Joystick(2);
    private BooleanSupplier intakeFullSupplier = () -> m_intake.getIntakeHasObject();
    public Trigger intakeFullTrigger = new Trigger(intakeFullSupplier);

    private LEDColorState ledColorState() {
        return LEDColorState.WHITE;
    }

    public RobotContainer() {
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

        intakeFullTrigger.whileTrue(
                new RepeatCommand(
                        Commands.sequence(
                                new InstantCommand(() -> m_led.setAllLedsStaticColorMode(255, 0, 0)),
                                new WaitCommand(0.2),
                                new InstantCommand(() -> m_led.setAllLedsStaticColorMode(0, 0, 0)),
                                new WaitCommand(0.2))));
        new JoystickButton(thirdJoystick, 1).whileTrue(new SetArmClosedLoop(m_arm, 190, true));

        new JoystickButton(thirdJoystick, 4).whileTrue(new SetArmClosedLoop(m_arm, -35, true));

        new JoystickButton(thirdJoystick, 6).whileTrue(new SetArmClosedLoop(m_arm, 90, true));

        new JoystickButton(thirdJoystick, 5).whileTrue(new SetArmClosedLoop(m_arm, 140, true));

        new JoystickButton(thirdJoystick, 2).whileTrue(new SetArmOpenLoop(m_arm, 0.04));
        new JoystickButton(thirdJoystick, 3).whileTrue(new SetArmOpenLoop(m_arm, -0.04));

        // * weirdo ripple effect */
        new JoystickButton(driver, 1)
                .toggleOnTrue(
                        new InstantCommand(() -> m_led.setRGB(0, 0, 254))
                                .andThen(
                                        new RepeatCommand(
                                                Commands.sequence(
                                                        new InstantCommand(() -> m_led.decreaseAllLedsBrightness()),
                                                        new WaitCommand(0),
                                                        new LighNextLed(m_led)))));

        // * blinking effect */
        new JoystickButton(driver, 3)
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
                                }));

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
                                }));

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
                                }));

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
                                }));

        // Commands.repeatingSequence(m_led.setAllLedsBlinking(255, 0, 0, 0.1)));

        // ! due to limited amount of buttons to bind, some arm angle commands should be done by

        // OPERATOR

        // ? led will be static color when lifting object to specific height to reduce distraction
        // * arm angle for cone actions */

        new JoystickButton(operator, 3)
                .onTrue(new SetArmClosedLoop(m_arm, ArmConstants.kArmConeCubeMid, true));

        // ! because of lack of buttons, intake control and human player is binded to operator
        // ? blinking led animation is used when picking up cone from ground and from human player
        new JoystickButton(operator, 1)
                .onTrue(new SetArmClosedLoop(m_arm, ArmConstants.kArmHome, true));

        new POVButton(operator, 180)
                .onTrue(new SetArmClosedLoop(m_arm, ArmConstants.kArmConeGround, true));

        new POVButton(operator, 90)
                .onTrue(new SetArmClosedLoop(m_arm, ArmConstants.kArmCubeGround, true));

        new POVButton(operator, 270).onTrue(new SetArmClosedLoop(m_arm, ArmConstants.kArmHuman, true));

        // * arm home command to set arm angle to 0 degrees */
        new JoystickButton(operator, 10).onTrue(new InstantCommand(() -> m_arm.resetArm(0)));

        new JoystickButton(operator, 2)
                .whileTrue(
                        new SetArmOpenLoop(m_arm, 0.7).withTimeout(1).raceWith(new IntakeStart(m_intake, 0.5)));

        // OUTTAKKKEE
        // CONE outtake
        new JoystickButton(operator, 5)
                .whileTrue(
                        Commands.parallel(
                                new InstantCommand(() -> m_led.setAllLedsStaticColorMode(0, 0, 0)),
                                new IntakeStart(m_intake, -0.5))); // TRIGGER EKLENECEK

        // CUBE outtake
        new JoystickButton(operator, 5)
                .whileTrue(
                        Commands.parallel(
                                new InstantCommand(() -> m_led.setAllLedsStaticColorMode(0, 0, 0)),
                                new IntakeStart(m_intake, 0.5))); // TRIGGER EKLENECeKKKK

        // ! intake is most vulnurable when enabled
        // ? leds will be rose color to indicate that intake is active
        // * intake controls are done by operator, having to do this would be too much for driver

        // CUBEEEEE INTAKE
        new JoystickButton(operator, 5)
                .whileTrue(
                        Commands.parallel(
                                new InstantCommand(() -> m_led.setAllLedsStaticColorMode(0, 0, 0)),
                                new IntakeStart(m_intake, 0.5)));

        // CONEEEEEE INTAKE
        new JoystickButton(operator, 6)
                .whileTrue(
                        Commands.parallel(
                                new InstantCommand(() -> m_led.setAllLedsStaticColorMode(0, 0, 0)),
                                new IntakeStart(m_intake, -0.5)));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
