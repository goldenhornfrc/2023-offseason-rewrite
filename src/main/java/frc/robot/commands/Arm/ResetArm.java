// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends InstantCommand {
  private static ArmSubsystem m_arm;
  private static double m_AngleTarget;

  public ResetArm(ArmSubsystem arm) {
    m_arm = arm;
    m_AngleTarget = ArmConstants.kArmResetSensorAngle;
  }

  public ResetArm(ArmSubsystem arm, double specificAngle){
    m_arm = arm;
    m_AngleTarget = specificAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.resetArm(m_AngleTarget);
  }
}
