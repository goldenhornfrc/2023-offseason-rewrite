// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmClosedLoop extends CommandBase {
    private ArmSubsystem m_arm;
    private double m_targetAngle;
    private boolean m_isContinous;

    /** Creates a new SetArmClosedLoop. */
    public SetArmClosedLoop(ArmSubsystem arm, double target, boolean iscontinious) {
        m_arm = arm;
        m_targetAngle = target;
        m_isContinous = iscontinious;
        addRequirements(m_arm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println(m_targetAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_arm.setArmAngle(m_targetAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.stopArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_isContinous) {
            return false;
        } else if ((Math.abs(m_arm.getArmAngle() - m_targetAngle)) <= ArmConstants.kArmTolerance) {
            return true;
        } else {
            return false;
        }
    }
}
