// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.IntakeConstants.intakeMotorID);
    private StatorCurrentLimitConfiguration m_limit = new StatorCurrentLimitConfiguration();

    /** Creates a new IntakeSubsystem. */
    public enum IntakeState {
        HOLD,
        RELEASE,
        STAND,
        RUN
    }

    private IntakeState intakeState = IntakeState.STAND;

    public IntakeSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(Constants.IntakeConstants.motorNeutralMode);
        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInverted);
        intakeMotor.configOpenloopRamp(0);
        // config open loop???
        m_limit.triggerThresholdCurrent = 0;
        m_limit.triggerThresholdTime = 0;
        m_limit.currentLimit = 0;
        m_limit.enable = true;
    }

    public void setMotor(double speed) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void stopMotor(double speed) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void IntakeState(IntakeState state) {
        intakeState = state;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setDefaultCurrentLimit() {
        intakeMotor.configStatorCurrentLimit(m_limit);
    }

    public void disableCurrentLimit() {
        intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
