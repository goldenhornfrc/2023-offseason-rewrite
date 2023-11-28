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
            new SimpleMotorFeedforward(
                    Constants.SwerveConstants.kA, Constants.SwerveConstants.kS, Constants.SwerveConstants.kV);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        // config
        angleEncoder = new CANCoder(moduleConstants.CANCoderID);
        configAngleEncoder();
        // config
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();
        // config
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

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
        double absolutePosition = getCANCoder().getDegrees() - angleOffset.getDegrees();
        System.out.println(
                absolutePosition + " " + integratedAngleEncoder.setPosition(absolutePosition).toString());
    }

    public void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInverted);
        driveMotor.setIdleMode(Constants.SwerveConstants.driveIdleMode);
        driveMotor.setSmartCurrentLimit(60);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.drivePositionFactor);
        driveController.setP(Constants.SwerveConstants.drivekP);
        driveController.setI(Constants.SwerveConstants.drivekI);
        driveController.setD(Constants.SwerveConstants.drivekD);
        driveController.setFF(Constants.SwerveConstants.drivekFF);
        driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageCompansationValue);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInverted);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleIdleMode);
        angleMotor.setSmartCurrentLimit(20);
        integratedAngleEncoder.setPositionConversionFactor(
                Constants.SwerveConstants.anglePositionFactor);
        angleController.setP(Constants.SwerveConstants.anglekP);
        angleController.setI(Constants.SwerveConstants.anglekI);
        angleController.setD(Constants.SwerveConstants.anglekD);
        angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageCompansationValue);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    // constants
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                        ? getAngle()
                        : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCANCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), getAngle());
    }
}
