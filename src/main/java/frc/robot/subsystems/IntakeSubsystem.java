// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.IntakeConstants.intakeMotorID);
    private SupplyCurrentLimitConfiguration m_limit = new SupplyCurrentLimitConfiguration();
    private boolean intakeHasObject = false;

    /** Creates a new IntakeSubsystem. */
    public enum IntakeState {
        HOLD,
        RELEASE,
        STANDBY,
        RUN
    }

    private IntakeState intakeState = IntakeState.STANDBY;

    public IntakeSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(Constants.IntakeConstants.motorNeutralMode);
        intakeMotor.setInverted(Constants.IntakeConstants.intakeMotorInverted);
        intakeMotor.configOpenloopRamp(Constants.IntakeConstants.configOpenloopRamp);
        // config open loop???
        m_limit.triggerThresholdCurrent = Constants.IntakeConstants.triggerThresholdCurrent;
        m_limit.triggerThresholdTime = Constants.IntakeConstants.triggerThresholdTime;
        m_limit.currentLimit = Constants.IntakeConstants.currentLimit;
        m_limit.enable = true;
    }

    public void setMotor(double speed) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void stopMotor(double speed) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public boolean getIntakeHasObject() {
        return intakeHasObject;
    }

    public void setIntakeHasObject(boolean value) {
        intakeHasObject = value;
    }

    public double getMotorCurrent() {

        return intakeMotor.getSupplyCurrent();
    }

    public void setDefaultCurrentLimit() {
        intakeMotor.configSupplyCurrentLimit(m_limit);
    }

    public void disableCurrentLimit() {
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration());
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }
}
