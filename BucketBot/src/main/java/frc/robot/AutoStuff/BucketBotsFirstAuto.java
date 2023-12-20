package frc.robot.AutoStuff;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class BucketBotsFirstAuto extends SequentialCommandGroup {
    public BucketBotsFirstAuto(SwerveSubsystem s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.ModuleConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory goForwardTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(2, 0)), 
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goForwardControllerCommand =
            new SwerveControllerCommand(
                goForwardTrajectory,
                s_Swerve::getPose,
                Constants.ModuleConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(new InstantCommand(()-> s_Swerve.resetOdometry(goForwardTrajectory.getInitialPose())), goForwardControllerCommand
    
        );
    }
}