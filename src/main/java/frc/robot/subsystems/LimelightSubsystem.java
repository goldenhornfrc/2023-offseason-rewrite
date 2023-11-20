// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    private static boolean m_isValid;
    private static double m_targetY, m_targetX;

    public LimelightSubsystem() {}

    @Override
    public void periodic() {
        m_isValid = LimelightHelpers.getTV("limelight");

        if (m_isValid) {
            m_targetX = LimelightHelpers.getTX("limelight");
            m_targetY = LimelightHelpers.getTY("limelight");
        }
    }

    public boolean isTargetValid() {
        return m_isValid;
    }

    public double getTargetY() {
        return m_targetY;
    }

    public double getTargetX() {
        return m_targetX;
    }

    public void setLedOn() {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    public void setLedOff() {
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }

    public double getDistance() {
        return ((LimelightConstants.kTargetHeight - LimelightConstants.kLimeLightHeight)
                / Math.tan(Math.toRadians(m_targetY)));
    }
}
