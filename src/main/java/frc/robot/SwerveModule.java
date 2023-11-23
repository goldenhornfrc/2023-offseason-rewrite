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
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;



/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integradetAngleEncoder;
    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(moduleNumber, moduleNumber, moduleNumber);

    public SwerveModule(int moduleNumber,SwerveModuleConstants moduleconstans){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleconstans.angleOffset;



        //config
        angleEncoder = new CANCoder(moduleconstans.cancoderID);
        configAngleEncoder();
        //config
        driveMotor = new CANSparkMax(moduleconstans.driveMotorID,MotorType.kBrushless);
        driveEncoder= driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        //config
        angleMotor = new CANSparkMax(moduleconstans.angleMotorID,MotorType.kBrushless);
        integradetAngleEncoder = angleMotor.getEncoder();
        angleController= angleMotor.getPIDController();

        lastAngle = getState().angle;
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder,CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    public void setDesiredState(SwerveModuleState desiredState,boolean isOpenLoop){
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

    }

    public void resetToAbsolute(){
        double absolutePosition = getAngle().getDegrees() - angleOffset.getDegrees();
        integradetAngleEncoder.setPosition(absolutePosition);
    }
    public void configDriveMotor(){
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(null);
        driveMotor.setSmartCurrentLimit(60);
        driveEncoder.getVelocityConversionFactor();
        driveEncoder.getPositionConversionFactor();
        driveController.getP();
        driveController.getI();
        driveController.getD();
        driveController.getFF();
        driveMotor.enableVoltageCompensation(12);
        driveEncoder.setPosition(0.0);
        driveMotor.burnFlash();

    }
    public void configAngleMotor(){
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor,Usage.kMinimal);
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(null);
        angleMotor.setSmartCurrentLimit(20);
        integradetAngleEncoder.setPositionConversionFactor(28);
        angleController.getP();
        angleController.getI();
        angleController.getD();
        angleController.getFF();
        angleMotor.enableVoltageCompensation(12);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void setSpeed(SwerveModuleState desiredState,boolean isOpenLoop){
        if(isOpenLoop){
            double percentoutput = desiredState.speedMetersPerSecond / 4.5;
            driveMotor.set(percentoutput);
        }else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,ControlType.kVelocity,0,feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (4.5 * 0.01)) ? getAngle() :desiredState.angle;

        angleController.setReference(angle.getDegrees(),ControlType.kPosition);
        lastAngle = angle;
    }


    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integradetAngleEncoder.getPosition());
    }
    private Rotation2d getCANCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }


    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(),getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveMotor.getEncoder().getPosition(),getAngle());
    }

}
