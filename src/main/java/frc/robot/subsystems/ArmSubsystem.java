// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(ArmConstants.kMotorID);

    public ArmSubsystem() {
        armMotor.configFactoryDefault();
        armMotor.setInverted(ArmConstants.kMotorInverted);
        armMotor.setNeutralMode(ArmConstants.kMotorNeutralMode);
        armMotor.configClosedloopRamp(ArmConstants.kArmClosedLoopRamp);

        armMotor.config_kP(0, ArmConstants.kArmP);
        armMotor.config_kI(0, ArmConstants.kArmI);
        armMotor.config_kD(0, ArmConstants.kArmD);

        armMotor.enableVoltageCompensation(true);
        armMotor.configVoltageCompSaturation(ArmConstants.kArmVoltageCompansationValue);

        armMotor.configForwardSoftLimitEnable(true);
        armMotor.configReverseSoftLimitEnable(true);
        armMotor.configForwardSoftLimitThreshold(ArmConstants.kArmTopLimit);
        armMotor.configReverseSoftLimitThreshold(ArmConstants.kArmBottomLimit);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setArmCoast() {
        armMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setArmBrake() {
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setArm(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopArm() {
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setArmAngle(double targetAngle) {
        armMotor.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        armMotor.configMotionAcceleration(ArmConstants.kArmAcceleration);

        armMotor.set(
                ControlMode.MotionMagic,
                (targetAngle / ArmConstants.kArmConversionFactor),
                DemandType.ArbitraryFeedForward,
                Math.cos(Math.toRadians(getArmAngle())) * ArmConstants.kArmFeedForwardValue);
    }

    public double getArmAngle() {
        return armMotor.getSelectedSensorPosition() * ArmConstants.kArmConversionFactor;
    }
    public void resetArm(double value) {
        armMotor.setSelectedSensorPosition(value);
    }
    public double getCurrentSensorPosition() {
        return armMotor.getSelectedSensorPosition();
    }
}
