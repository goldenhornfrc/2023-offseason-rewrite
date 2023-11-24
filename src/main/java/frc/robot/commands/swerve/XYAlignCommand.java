// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class XYAlignCommand extends CommandBase {
  private final Swerve m_Swerve;
  private final LimelightSubsystem m_lime;
  private double output;
  private int allianceModifier = 1;
  /** Creates a new XYAlignCommand. */
  public XYAlignCommand(Swerve swerve,LimelightSubsystem limelight) {
    m_Swerve = swerve;
    m_lime = limelight;
    addRequirements(m_Swerve);
    addRequirements(m_lime);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    output = 0.0;
    m_Swerve.getXYController().setTolerance(Constants.SwerveConstants.pidConstants.xyControllerTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.getAlliance() == Alliance.Red) {allianceModifier = -1;}
    else{
     allianceModifier = 1;

    output = m_Swerve.getXYController().calculate(m_lime.getTargetY(),0);
    m_Swerve.drive(new Translation2d(0, output *allianceModifier), 0, true, true);

     
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Swerve.getXYController().atSetpoint() || m_lime.isTargetValid() ==true;
  }
}
