package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.ModuleConstants.angleEnableCurrentLimit, 
            Constants.ModuleConstants.angleContinuousCurrentLimit, 
            Constants.ModuleConstants.anglePeakCurrentLimit, 
            Constants.ModuleConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.ModuleConstants.kPTurning;
        swerveAngleFXConfig.slot0.kI = Constants.ModuleConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.ModuleConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.ModuleConstants.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.ModuleConstants.driveEnableCurrentLimit, 
            Constants.ModuleConstants.driveContinuousCurrentLimit, 
            Constants.ModuleConstants.drivePeakCurrentLimit, 
            Constants.ModuleConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.ModuleConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.ModuleConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.ModuleConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.ModuleConstants.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.ModuleConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.ModuleConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.ModuleConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}