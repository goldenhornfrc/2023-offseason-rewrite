package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(moduleNumber, moduleNumber, moduleNumber);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleconstans) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleconstans.angleOffset;

        // config
        angleEncoder = new CANCoder(moduleconstans.CANCoderID);
        configAngleEncoder();
        // config
        driveMotor = new CANSparkMax(moduleconstans.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        // config
        angleMotor = new CANSparkMax(moduleconstans.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();

        lastAngle = getState().angle;
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void resetToAbsolute() {
        double absolutePosition = getAngle().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    public void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(null);
        driveMotor.setSmartCurrentLimit(60);
        driveEncoder.setVelocityConversionFactor(0.0);
        driveEncoder.setPositionConversionFactor(0.0);
        driveController.setP(0.0);
        driveController.setI(0.0);
        driveController.setD(0.0);
        driveController.setFF(0.0);
        driveMotor.enableVoltageCompensation(12);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kMinimal);
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(null);
        angleMotor.setSmartCurrentLimit(20);
        integratedAngleEncoder.setPositionConversionFactor(28);
        angleController.setP(0.0);
        angleController.setI(0.0);
        angleController.setD(0.0);
        angleController.setFF(0.0);
        angleMotor.enableVoltageCompensation(12);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentoutput = desiredState.speedMetersPerSecond / 4.5;
            driveMotor.set(percentoutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (4.5 * 0.01))
                        ? getAngle()
                        : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    private Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getAngle());
    }
}
