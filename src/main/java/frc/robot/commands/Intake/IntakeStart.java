// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class IntakeStart extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final double m_speed;

    /** Creates a new IntakeStart. */
    public IntakeStart(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        m_speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_intake.getIntakeState() != IntakeState.HOLD) {
            m_intake.setIntakeState(IntakeState.RUN);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_intake.getMotorCurrent() >= IntakeConstants.objectDetectionCurrent
                && m_intake.getIntakeState() == IntakeState.RUN) {
            m_intake.setIntakeState(IntakeState.HOLD);
            m_intake.setIntakeHasObject(true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (m_intake.getIntakeState() == IntakeState.HOLD) {
            m_intake.setMotor((m_speed / Math.abs(m_speed)) * 0.3);
        } else {
            m_intake.setMotor(0);
            m_intake.setIntakeState(IntakeState.STANDBY);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
