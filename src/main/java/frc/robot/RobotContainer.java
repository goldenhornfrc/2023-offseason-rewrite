// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    public final Swerve m_swerve = new Swerve();
    public final LimelightSubsystem m_lime = new LimelightSubsystem();
    private final Joystick driver = new Joystick(0);

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

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
