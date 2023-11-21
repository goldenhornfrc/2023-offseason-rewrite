// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends CommandBase {
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.resetArm(m_AngleTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_arm.getCurrentSensorPosition() == m_AngleTarget);
  }
}
