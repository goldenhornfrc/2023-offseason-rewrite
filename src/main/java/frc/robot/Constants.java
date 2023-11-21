// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
}
