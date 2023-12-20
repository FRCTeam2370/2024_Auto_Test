// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadBand = 0.1;
    public static final double xLimiter = 1;
    public static final double yLimiter = 1;
    public static final double rotationLimiter = 1;
  }
  public static TalonFXConfiguration turningMotorConfiguration;
  public static TalonFXConfiguration drivingMotorConfiguration;



  public static final class ModuleConstants{
    
    public static final double feedforwardkS = (0.32 / 12);
    public static final double feedforwardkV = (1.51 / 12);
    public static final double feedforwardkA = (0.27 / 12);
    

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
    public static final double kMaxDriveSpeedMetersPerSecond = 5;
    public static final double kMaxRotationSpeedMetersPerSecond = 2;

    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double driveGearRatio = (5.14 / 1.0);
    /** 12.8 : 1 */
    public static final double rotationGearRatio = (12.8 / 1.0);

    public static final double driveEncoderRotationToMeter = driveGearRatio * Math.PI * wheelDiameter;
    public static final double rotationEncoderToRadian = rotationGearRatio * 2 * Math.PI;
    public static final double driveEncoderRPMToMeterPerSecond = driveEncoderRotationToMeter / 60;
    public static final double rotatoinEncoderRPMToRadianPerSecond = rotationEncoderToRadian / 60;

    public static final double kPTurning = 0.2;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKF = 0.0;

    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = false;
    public static final boolean canCoderInvert = false;
   

    public static final int kFrontRightDriveID = 36;
    public static final int kFrontRightTurnID = 37;
    public static final int kFrontRightCANCoder = 38;
    public static final boolean kFrontRightDriveReversed = false;
    public static final boolean kFrontRightTurnReversed = false;
    public static final boolean kFrontRightCANCoderReversed = false;
    public static final Rotation2d kFrontRightCANCoderOffsetRadians = Rotation2d.fromDegrees(128.672);
    

    public static final int kFrontLeftDriveID = 33;
    public static final int kFrontLeftTurnID = 34;
    public static final int kFrontLeftCANCoder = 35;
    public static final boolean kFrontLeftDriveReversed = false;
    public static final boolean kFrontLeftTurnReversed = false;
    public static final boolean kFrontLeftCANCoderReversed = false;
    public static final Rotation2d kFrontLeftCANCoderOffsetRadians = Rotation2d.fromDegrees(277.471);


    public static final int kBackRightDriveID = 27;
    public static final int kBackRightTurnID = 28;
    public static final int kBackRightCANCoder = 29;
    public static final boolean kBackRightDriveReversed = false;
    public static final boolean kBackRightTurnReversed = false;
    public static final boolean kBackRightCANCoderReversed = false;
    public static final Rotation2d kBackRightCANCoderOffsetRadians = Rotation2d.fromDegrees(357.275);


    public static final int kBackLeftDriveID = 30;
    public static final int kBackLeftTurnID = 31;
    public static final int kBackLeftCANCoder = 32;
    public static final boolean kBackLeftDriveReversed = false;
    public static final boolean kBackLeftTurnReversed = false;
    public static final boolean kBackLeftCANCoderReversed = false;
    public static final Rotation2d kBackLeftCANCoderOffsetRadians = Rotation2d.fromDegrees(135.791);

    public static final int Pigeon2ID = 1;

    public static final double kTrackWidth = Units.inchesToMeters(11.50);
    public static final double kWheelBase = Units.inchesToMeters(11.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, - kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    );
    

     /* Swerve Current Limiting */
     public static final int angleContinuousCurrentLimit = 25;
     public static final int anglePeakCurrentLimit = 40;
     public static final double anglePeakCurrentDuration = 0.1;
     public static final boolean angleEnableCurrentLimit = true;

     public static final int driveContinuousCurrentLimit = 35;
     public static final int drivePeakCurrentLimit = 60;
     public static final double drivePeakCurrentDuration = 0.1;
     public static final boolean driveEnableCurrentLimit = true;

     public static final double driveKP = 0.2; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
  }
  
  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 7;
    public static final double kMaxAccelerationMetersPerSecondSquared = 7;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI*2;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 15;
    public static final double kDThetaController = 0;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}
}
