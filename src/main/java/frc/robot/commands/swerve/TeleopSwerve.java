// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
    private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
    private Swerve s_Swerve;
    private LimelightSubsystem m_Limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSup;
    public int allianceModifier = 1;

    /** Creates a new TeleopSwerve. */
    public TeleopSwerve(
            Swerve s_Swerve,
            LimelightSubsystem lime,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier fieldCentricSup) {
        this.s_Swerve = s_Swerve;
        m_Limelight = lime;
        addRequirements(s_Swerve);
        // Use addRequirements() here to declare subsystem dependencies.

        this.rotationSup = rotationSup;
        this.strafeSup = strafeSup;
        this.fieldCentricSup = fieldCentricSup;
        this.translationSup = translationSup;

        xLimiter = new SlewRateLimiter(Constants.SwerveConstants.xLimiter);
        yLimiter = new SlewRateLimiter(Constants.SwerveConstants.yLimiter);
        rotLimiter = new SlewRateLimiter(Constants.SwerveConstants.rotLimiter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        s_Swerve
                .getRotController()
                .setTolerance(Constants.SwerveConstants.pidConstants.rotControllerTolerance);
        s_Swerve.getRotController().enableContinuousInput(-180, 180);
        s_Swerve
                .getXYController()
                .setTolerance(Constants.SwerveConstants.pidConstants.xyControllerTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(rotationSup.getAsDouble()) >= 0.1) {
            s_Swerve.setWantsHeadingLock(false);
        }

        double translationVal =
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        translationVal = xLimiter.calculate(translationVal) * Constants.SwerveConstants.maxSpeed;
        rotationVal = rotLimiter.calculate(rotationVal) * Constants.SwerveConstants.angularMaxVelocity;
        strafeVal = yLimiter.calculate(strafeVal) * Constants.SwerveConstants.maxSpeed;

        if (Robot.getAlliance() == Alliance.Red) {
            allianceModifier = -1;
        } else {
            allianceModifier = 1;
        }

        if (s_Swerve.getWantsHeadingLock()) {
            rotationVal =
                    s_Swerve
                            .getRotController()
                            .calculate(
                                    s_Swerve.getPose().getRotation().getDegrees(),
                                    s_Swerve.getHeadingLockTargetAngle());
        }

        if (s_Swerve.getWantsVisionAlign()) {
            if (m_Limelight.isTargetValid()) {
                strafeVal = s_Swerve.getXYController().calculate(-m_Limelight.getTargetY(), 0);
                translationVal = translationVal * 0.5;
            }
        }

        s_Swerve.drive(
                new Translation2d(translationVal * allianceModifier, strafeVal * allianceModifier),
                rotationVal,
                fieldCentricSup.getAsBoolean(),
                true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
