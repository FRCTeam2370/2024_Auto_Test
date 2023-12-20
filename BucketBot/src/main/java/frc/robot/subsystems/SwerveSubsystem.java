// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.SwerveModuleClass;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

    private final static SwerveModuleClass frontRight = new SwerveModuleClass(
      0,
      ModuleConstants.kFrontRightDriveID,
      ModuleConstants.kFrontRightTurnID,
      ModuleConstants.kFrontRightDriveReversed,
      ModuleConstants.kFrontRightTurnReversed,
      ModuleConstants.kFrontRightCANCoder,
      ModuleConstants.kFrontRightCANCoderOffsetRadians,
      ModuleConstants.kFrontRightCANCoderReversed
    );

    private final static SwerveModuleClass frontLeftt = new SwerveModuleClass(
      1,
      ModuleConstants.kFrontLeftDriveID,
      ModuleConstants.kFrontLeftTurnID,
      ModuleConstants.kFrontLeftDriveReversed,
      ModuleConstants.kFrontLeftTurnReversed,
      ModuleConstants.kFrontLeftCANCoder,
      ModuleConstants.kFrontLeftCANCoderOffsetRadians,
      ModuleConstants.kFrontLeftCANCoderReversed
    );

    private final static SwerveModuleClass BackRight = new SwerveModuleClass(
      2,
      ModuleConstants.kBackRightDriveID,
      ModuleConstants.kBackRightTurnID,
      ModuleConstants.kBackRightDriveReversed,
      ModuleConstants.kBackRightTurnReversed,
      ModuleConstants.kBackRightCANCoder,
      ModuleConstants.kBackRightCANCoderOffsetRadians,
      ModuleConstants.kBackRightCANCoderReversed
    );

    private final static SwerveModuleClass BackLeft = new SwerveModuleClass(
      3,
      ModuleConstants.kBackLeftDriveID,
      ModuleConstants.kBackLeftTurnID,
      ModuleConstants.kBackLeftDriveReversed,
      ModuleConstants.kBackLeftTurnReversed,
      ModuleConstants.kBackLeftCANCoder,
      ModuleConstants.kBackLeftCANCoderOffsetRadians,
      ModuleConstants.kBackLeftCANCoderReversed
    );

    private static Pigeon2 gyro = new Pigeon2(Constants.ModuleConstants.Pigeon2ID);
    //The odometer object is declared here but later defined in the SwerveSubsystem main
    public SwerveDriveOdometry odometer;
    public SwerveModuleClass[] mSwerveMods;
    
    public static void zeroGyro(){
      gyro.setYaw(0);
    }

    public double getHeading(){
      return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public Rotation2d geRotation2d(){
      return Rotation2d.fromDegrees(gyro.getYaw());
    }
    public SwerveSubsystem() {

      mSwerveMods = new SwerveModuleClass[] {
        frontRight, frontLeftt, BackRight, BackLeft
      };

      SmartDashboard.putNumber("", getHeading());
      gyro.configFactoryDefault();
      zeroGyro();
      //this is the odometer object that will later be used in autonomous
      odometer = new SwerveDriveOdometry(ModuleConstants.kDriveKinematics, new Rotation2d(0), getModulePositions());

    }

    //The following two methods are used with the odometer. 
    //One is for reseting the odometer and the other to get the current possition on the field
    public Pose2d getPose(){
      return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
      odometer.resetPosition(geRotation2d(), getModulePositions(), pose);
    }

  
  
//This method is used in the definition of the odometer object. 
//It gets the positions of all the different modules
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      frontRight.getPosition(), frontLeftt.getPosition(), BackRight.getPosition(), BackLeft.getPosition()
    };
    return positions;
  }

  public void stopModules(){
    frontLeftt.stop();
    frontRight.stop();
    BackLeft.stop();
    BackRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeftt.setDesiredStates(desiredStates[0]);
    frontRight.setDesiredStates(desiredStates[1]);
    BackLeft.setDesiredStates(desiredStates[2]);
    BackRight.setDesiredStates(desiredStates[3]);
  }

  public void setModulesToAbsolute(){
    frontLeftt.resetMotorEncoders();
    frontRight.resetMotorEncoders();
    BackLeft.resetMotorEncoders();
    BackRight.resetMotorEncoders();
  }

  

  public static double getRotationPosition(int module){
    if(module == 0){
      return frontLeftt.getTurningPosition();
    }else if(module == 1){
      return frontRight.getTurningPosition();
    }else if(module == 2){
      return BackLeft.getTurningPosition();
    }else if(module == 3){
      return BackRight.getTurningPosition();
    }else{
      return 0;
    }
    
  }
  @Override
  public void periodic() {
    odometer.update(geRotation2d(), getModulePositions()); 
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading/Direction", getHeading());
    SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putString("Module Positions", getModulePositions().toString());
  }
}
