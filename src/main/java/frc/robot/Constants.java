// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
    }

    public static class LimelightConstants {
        public static final double kLimeLightHeight = 60.0;
        public static final double kTargetHeight = 43.0;
    }

    public static class LEDConstants {
        public static final int kLEDHeader = 0;
        public static final int kLEDBuffer = 60;
    }

    //drivetrain
    public static final class Swerve{
        public static final boolean canCoderInvert = false;
        public static final double trackWidth = 0.5857;
        public static final double wheelBase = 0.5857;
        public static final double wheelDiameter = Units.inchesToMeters(3.95);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
    
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1
    
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    

    }
    
}
