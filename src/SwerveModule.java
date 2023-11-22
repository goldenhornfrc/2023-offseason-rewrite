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
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;



/** Add your docs here. */
public class SwerveModule {
    private int moduleNumber;
    private Rotation2d lastangle;
    private Rotation2d angleOffsett;

    private CANSparkMax anglemotor;
    private CANSparkMax drivemotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integradetAngleEncoder;
    private CANCoder angleEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController angleController;

    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(moduleNumber, moduleNumber, moduleNumber);

    public SwerveModule(int moduleNumber,SwerveModuleConstants moduleconstans){
        this.moduleNumber = moduleNumber;
        angleOffsett = moduleconstans.angleOffsett;



        //config
        angleEncoder = new CANCoder(moduleconstans.canCoderID);
        configangleEncoder();
        //config
        drivemotor = new CANSparkMax(moduleconstans.driveMotorID,MotorType.kBrushless);
        driveEncoder= drivemotor.getEncoder();
        driveController = drivemotor.getPIDController();
        //config
        anglemotor = new CANSparkMax(moduleconstans.angleMotorID,MotorType.kBrushless);
        integradetAngleEncoder = anglemotor.getEncoder();
        angleController= anglemotor.getPIDController();

        lastangle = getState().angle;
    }

    private void configangleEncoder() {
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder,CCUsage.kMinimal);
        angleEncoder.configAllSettings(null);
    }

    public void setDesiredState(SwerveModuleState desiredState,boolean isOpenLoop){
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

    }

    public void resetToAbsolute(){    
        double absolutePosition = getAngle().getDegrees() - angleOffsett.getDegrees();
        integradetAngleEncoder.setPosition(absolutePosition);
    }
    public void configDriveMotor(){
        drivemotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(drivemotor, Usage.kAll);
        drivemotor.getInverted();
        drivemotor.getIdleMode();
        drivemotor.setSmartCurrentLimit(60);
        driveEncoder.getVelocityConversionFactor();
        driveEncoder.getPositionConversionFactor();
        driveController.getP();
        driveController.getI();
        driveController.getD();
        driveController.getFF();
        drivemotor.burnFlash();
        drivemotor.enableVoltageCompensation(12);
        driveEncoder.setPosition(0.0);
    }
    public void configAngleMotor(){
        anglemotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(anglemotor, Usage.kMinimal);
        anglemotor.getInverted();
        anglemotor.getIdleMode();
        anglemotor.setSmartCurrentLimit(20);
        integradetAngleEncoder.setPositionConversionFactor(28);
        angleController.getP();
        angleController.getI();
        angleController.getD();
        angleController.getFF();
        anglemotor.enableVoltageCompensation(12);
        anglemotor.burnFlash();
        resetToAbsolute();
    }

    private void setSpeed(SwerveModuleState desiredState,boolean isOpenLoop){
        if(isOpenLoop){
            double percentoutput = desiredState.speedMetersPerSecond / 4.5;
            drivemotor.set(percentoutput);
        }else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,ControlType.kVelocity,0,feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (4.5 * 0.01)) ? getAngle() :desiredState.angle;

        angleController.setReference(angle.getDegrees(),ControlType.kPosition);
        lastangle = angle;
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
        return new SwerveModulePosition(drivemotor.getEncoder().getPosition(),getAngle());
    }

}
