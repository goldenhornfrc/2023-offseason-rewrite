// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

/**
  * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
  * constants. This class should not be used for any other purpose. All constants should be declared
  * globally (i.e. public static). Do not put anything functional in this class.
  *
  * <p>It is advised to statically import this class (or one of its inner classes) wherever the
  * constants are needed, to reduce verbosity.
  */
public final class Constants {
    public static final double stickDeadband = 0.1;

    public static class ArmConstants {
        public static final int kMotorID = 30;
        public static final boolean kMotorInverted = false;
        public static final NeutralMode kMotorNeutralMode = NeutralMode.Brake;
        public static final double kArmClosedLoopRamp = 1;

        public static final double kArmConversionFactor = ((48.0 * 2.8) / 2048.0) * 360.0;

        public static final double kArmTopLimit = 0.0;
        public static final double kArmBottomLimit = 0.0;
        public static final double kArmVoltageCompansationValue = 12.0;

        public static final double kArmCruiseVelocity = 0.0;
        public static final double kArmAcceleration = 0.0;

        public static final double kArmP = 0.0;
        public static final double kArmI = 0.0;
        public static final double kArmD = 0.0;
        public static final double kArmFeedForwardValue = 0.0;

        public static final double kArmTolerance = 1;

        public static final double kArmResetSensorAngle = 0.0;

        // TODO: put in correct numbers here
        public static final double kArmConeGround = 0;
        public static final double kArmCubeGround = 0;
        public static final double kArmHome = 0;
        public static final double kArmConeMid = 0;
        public static final double kArmCubeMid = 0;
        public static final double kArmHuman = 0;
    }

    public static class LimelightConstants {
        public static final double kLimeLightHeight = 60.0;
        public static final double kTargetHeight = 43.0;
    }

    public static class LEDConstants {
        public static final int kLEDHeader = 0;
        public static final int kLEDBuffer = 18;
    }

    public static class IntakeConstants {
        public static final int intakeMotorID = 25;
        public static final NeutralMode motorNeutralMode = NeutralMode.Brake;
        public static final boolean intakeMotorInverted = false;
        public static final double triggerThresholdCurrent = 0;
        public static final double triggerThresholdTime = 0;
        public static final double currentLimit = 0;
        public static final double configOpenloopRamp = 0;
        public static final double objectDetectionCurrent = 5;
    }

    // drivetrain
    public static final class SwerveConstants {
        public static final boolean canCoderInvert = false;
        public static final double trackWidth = 0.5857;
        public static final double wheelBase = 0.5857;
        public static final double wheelDiameter = Units.inchesToMeters(3.95);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0);
        public static final double angleGearRatio = (12.8 / 1.0);

        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        public static final IdleMode angleIdleMode = IdleMode.kBrake;

        public static final boolean invertGyro = true;

        public static final class pidConstants {
            public static final double rotControllerTolerance = 0;
            public static final double xyControllerTolerance = 0;
        }

        public static final double xLimiter = 0;
        public static final double yLimiter = 0;
        public static final double rotLimiter = 0;

        public static final boolean angleMotorInverted = false;
        public static final boolean driveMotorInverted = false;

        public static final double drivePositionFactor = wheelDiameter * Math.PI / driveGearRatio;
        public static final double driveVelocityFactor = drivePositionFactor / 60.0;

        public static final double anglePositionFactor = 360.0 / angleGearRatio;

        public static final double anglekP = 0.01907;
        public static final double anglekI = 0;
        public static final double anglekD = 0.0031;

        public static final double drivekP = 0.0;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekFF = 0;

        public static final double kV = 0;
        public static final double kS = 0;
        public static final double kA = 0;

        public static final double maxSpeed = 4.5;
        public static final double angularMaxVelocity = 11.5;

        public static final double rotControllerkP = 0;
        public static final double rotControllerkI = 0.0;
        public static final double rotControlerkD = 0;

        public static final double xyControllerkP = 0;
        public static final double xyControllerkI = 0.0;
        public static final double xyControllerkD = 0;

        public static final double voltageCompansationValue = 12;

        public static final SwerveDriveKinematics swerveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(192.92);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(349.37);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 18;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(276.77 - 180.0);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 20;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(49.31);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
