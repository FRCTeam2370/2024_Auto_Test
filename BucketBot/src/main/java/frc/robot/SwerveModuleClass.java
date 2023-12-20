// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModuleClass { 
    public int moduleNumber;
    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private Rotation2d lastAngle;
    private CANCoder rotationEncoder;

    private boolean absolutEncoderReversed;
    private Rotation2d absoluteEncoderOffset;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ModuleConstants.feedforwardkA, Constants.ModuleConstants.feedforwardkS
    , Constants.ModuleConstants.feedforwardkV);

    public SwerveModuleClass(int moduleNumber,int driveMotorId, int turnMotorId, boolean DriveMotorReversed, boolean TurningMotorReversed, int absoluteEncoderId, 
        Rotation2d absoluteEncoderOffset, boolean absoluteEncoderReversed){
        this.moduleNumber = moduleNumber;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absolutEncoderReversed = absoluteEncoderReversed;
        rotationEncoder = new CANCoder(absoluteEncoderId);
        rotationEncoder.configFactoryDefault();
        rotationEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        
        driveMotor = new TalonFX(driveMotorId);
        turnMotor = new TalonFX(turnMotorId);

        rotationEncoder.getAbsolutePosition();
        

        driveMotor.setInverted(DriveMotorReversed);
        turnMotor.setInverted(TurningMotorReversed);

        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);

        turnMotor.configFactoryDefault();
        turnMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        lastAngle = getState().angle;
        
        resetMotorEncoders();
        
        System.out.println("-----------------A----------------------");
        SmartDashboard.putNumber("Driving Motor", getDrivePosition());
        SmartDashboard.putNumber("Turning Motor", getTurningPosition());
    }

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition();
    }
       public double getTurningPosition(){
        double turnPose = turnMotor.getSelectedSensorPosition();
        return Conversions.falconToDegrees(turnPose, Constants.ModuleConstants.rotationGearRatio);
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity(){
        return turnMotor.getSelectedSensorVelocity();
    }

    public Rotation2d getCANCoder(){
        return Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition());
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(turnMotor.getSelectedSensorPosition(), 
        Constants.ModuleConstants.rotationGearRatio));
    
    }

    public void resetMotorEncoders(){
        double absolutePosistion = Conversions.degreesToFalcon(getCANCoder().getDegrees() - absoluteEncoderOffset.getDegrees(), 
        Constants.ModuleConstants.rotationGearRatio);   

        turnMotor.setSelectedSensorPosition(absolutePosistion);
        System.out.println("---------Encoders Reset---------");
    }

   

    public void setDesiredStates(SwerveModuleState state){
        state = CTREModuleState.optimize(state, getState().angle);
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.ModuleConstants.kMaxRotationSpeedMetersPerSecond * 0.01)) ? lastAngle : state.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
        double d_angle = Conversions.degreesToFalcon(angle.getDegrees(), Constants.ModuleConstants.rotationGearRatio);
        turnMotor.set(ControlMode.Position, d_angle);
        lastAngle = angle;
    
        double percentOutput = state.speedMetersPerSecond / Constants.ModuleConstants.kMaxDriveSpeedMetersPerSecond;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
      }

    public void stop(){
        driveMotor.set(ControlMode.Velocity, 0);
        turnMotor.set(ControlMode.Velocity, 0);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.ModuleConstants.wheelDiameter, Constants.ModuleConstants.driveGearRatio), 
            getAngle()
        );
    }

    public SwerveModuleState getState(){
         return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
            Constants.ModuleConstants.rotationGearRatio,
            Constants.ModuleConstants.driveGearRatio), 
            getAngle()
        ); 
    }
}