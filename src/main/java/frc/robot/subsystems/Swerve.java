package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;


public class Swerve extends SubsystemBase {
  public AHRS gyro;
  private final SwerveDrivePoseEstimator estimator;
  private static boolean isFieldOriented = true;
  private static boolean WantsHeadingLock = false;
  private static boolean WantsVisionAllign = false;
  private final SwerveModule[] mSwerveMods;
  public double HeadingLockTargetAngle;
  
  private Field2d field;



  private static final PIDController rotController = new PIDController(0
  , 0
  , 0);

  private static final PIDController xyController = new PIDController(0
  , 0
  , 0);

  
  public PIDController getrotController(){
    return rotController;
  }
  public PIDController getxyController(){
    return xyController;
  }

  public double getHeadingLockTargetAngle(){
    return HeadingLockTargetAngle;
  }

  public void setHeadingLockTargetAngle(double newAngle){
    if(Robot.getAlliance() == Alliance.Red){
      newAngle=(newAngle+180) %360;
    }
    HeadingLockTargetAngle = newAngle;
  }

  public boolean getWantsHeadingLock(){
    return WantsHeadingLock;
  }
  public void setWantsHeadingLock(boolean newHeadingLock){
     WantsHeadingLock = newHeadingLock;
    
  }

  public boolean getWantsVisionAllign(){
    return WantsVisionAllign;
  }
  public void setWantsVisionAllign(boolean newVisionAllign){
      WantsHeadingLock = newVisionAllign;
  }



  /** Ceates a new Swerve. */
  public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();

        mSwerveMods =
        new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
                    new SwerveModule(1, Constants.Swerve.Mod1.constants),
                    new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
       
       // desired State ,angular velocity,openloop,vecbuilder,IEEEremainder

        estimator =
            new SwerveDrivePoseEstimator(
                null,
                getHeadingRotation(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01)); 
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }



  public void drive(Translation2d translation,double rotation,boolean fieldRelative,boolean isOpenLoop){
    SwerveModuleState[] swerveModuleState = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
       fieldRelative ? 
       ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),translation.getY(), rotation ,  getPose().getRotation()) 
       :new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
       SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState, 4.5);
       //constant eklencek
       for(SwerveModule mod: mSwerveMods){
        mod.setDesiredState(swerveModuleState[mod.moduleNumber], isOpenLoop);
       }
  }

 
 
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4.5);
     //constants
     for(SwerveModule mod: mSwerveMods){
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
     }
  }
  public Pose2d getPose(){
    return estimator.getEstimatedPosition();
  }
  public void resetOdometry(Pose2d pose){
    estimator.resetPosition(getHeadingRotation(), getModulePositions(), pose);
  }
  public void EstimatorWithVision(Pose2d measurement,double latency){
    estimator.addVisionMeasurement(measurement,Timer.getFPGATimestamp() - latency);
  }
  public Rotation2d getHeadingRotation() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360) *(true ? -1.0 : 1.0));
  }
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod: mSwerveMods) {
    positions[mod.moduleNumber] = mod.getPosition();
    }return positions;
  }
  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] state = new SwerveModuleState[4];
    for(SwerveModule mod:mSwerveMods) {
      state[mod.moduleNumber] = mod.getState();

    }return state;
  }



  @Override
  public void periodic() {
    estimator.update(getHeadingRotation(), getModulePositions());
    field.setRobotPose(getPose());
  }


  public void zeroGyro(){
    gyro.zeroYaw();
  }
  public Rotation2d getGyroPitch(){
    return Rotation2d.fromDegrees(gyro.getPitch())
    ;
  }
  public Rotation2d getYaw(){
    return(true)?
    Rotation2d.fromDegrees(360 - gyro.getYaw()):
    Rotation2d.fromDegrees(gyro.getYaw());
  }
  public boolean getIsFieldOriented(){
    return isFieldOriented;
  }
  public void setisFieldOrieted(){
    isFieldOriented = true;
  }
  public void isRobotOriented(){
    isFieldOriented = false;
  }
}
