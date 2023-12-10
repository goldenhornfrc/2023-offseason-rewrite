package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class RotAlignCommand extends CommandBase {
    private final Swerve m_swerve;
    private final LimelightSubsystem m_lime;
    private double output;

    public RotAlignCommand(Swerve swerve, LimelightSubsystem limelight) {
        m_swerve = swerve;
        m_lime = limelight;
        addRequirements(m_swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        output = 0.0;
        m_swerve
                .getRotController()
                .setTolerance(Constants.SwerveConstants.pidConstants.rotControllerTolerance);

        m_swerve.getRotController().reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        output =
                m_swerve
                        .getRotController()
                        .calculate(
                                m_swerve.getHeadingRotation().getDegrees(),
                                Robot.getAlliance() == Alliance.Blue ? 0.0 : 180.0);
        m_swerve.drive(new Translation2d(0, 0), output, true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new Translation2d(0, 0), 0, true, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_swerve.getRotController().atSetpoint() || m_lime.isTargetValid() == true;
    }
}
