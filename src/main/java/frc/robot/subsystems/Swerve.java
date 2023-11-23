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


public class Swerve extends SubsystemBase {
  public AHRS gyro;
  private final SwerveDrivePoseEstimator estimator;
  private static boolean isFieldOriented = true;
  private static boolean WantsHeadingLock = false;
  private static boolean WantsVisionAllign = false;
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


  private SwerveModulePosition[] getModulePositions() {
    return null;
  }
  private Rotation2d getHeadingRotation() {
    return null;
  }
  public void drive(Translation2d translation,double rotation,boolean fieldrelative,boolean isOpenLoop){
   


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }




  public void zeroGyro(){
    gyro.zeroYaw();

  }
}
