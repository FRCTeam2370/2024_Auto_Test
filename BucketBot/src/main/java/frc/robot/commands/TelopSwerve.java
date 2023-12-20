// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TelopSwerve extends CommandBase {
  /** Creates a new TelopSwerve. */

  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpeedFunction, ySpeedFunction, rotationSpeedFuction;
  private final Boolean feildOriented;
  //private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

  public TelopSwerve(SwerveSubsystem s_swerveSubsystem, DoubleSupplier xSpeedFunction, DoubleSupplier ySpeedFunction, DoubleSupplier rotationSpeedFunction,
  Boolean feildOriented) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.swerveSubsystem = s_swerveSubsystem;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunction = ySpeedFunction;
    this.rotationSpeedFuction = rotationSpeedFunction;
    this.feildOriented = feildOriented;
    //this.xLimiter = new SlewRateLimiter(OperatorConstants.xLimiter);
    //this.yLimiter = new SlewRateLimiter(OperatorConstants.yLimiter);
    //this.rotationLimiter = new SlewRateLimiter(OperatorConstants.rotationLimiter);
    addRequirements(s_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.zeroGyro();
    System.out.println("--------Gyro Reset in Command---------");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = xSpeedFunction.getAsDouble();
    double ySpeed = ySpeedFunction.getAsDouble();
    double rotationSpeed = rotationSpeedFuction.getAsDouble();

    xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadBand ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadBand ? ySpeed : 0;
    rotationSpeed = Math.abs(rotationSpeed) > OperatorConstants.kDeadBand ? rotationSpeed : 0;

    xSpeed *= Constants.ModuleConstants.kMaxDriveSpeedMetersPerSecond;
    ySpeed *= Constants.ModuleConstants.kMaxDriveSpeedMetersPerSecond;
    rotationSpeed *= Constants.ModuleConstants.kMaxRotationSpeedMetersPerSecond * 1.5;

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotationSpeed", rotationSpeed);
    
    
    ChassisSpeeds chassisSpeeds;
    
    if(feildOriented == true){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, -ySpeed, -rotationSpeed, swerveSubsystem.geRotation2d());
    }else{
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    }

    SwerveModuleState[] moduleStates =  Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
