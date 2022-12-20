// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

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

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(SwerveDrive swerveDrive){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.kSwerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                swerveDrive::getPose,
                Constants.Swerve.kSwerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                // add a supplier here for desired rotation state at each time step, i.e. swerveDrive::getTrajectoryRotation
                swerveDrive::setModuleStates,
                swerveDrive);


        PathPlannerTrajectory ballBlueLeft = PathPlanner.loadPath("TopMiddle", 1.0, 1.0);

        var pathPlannerCommand =
            new AutoSwerveController(
                ballBlueLeft,
                swerveDrive::getPose,
                Constants.Swerve.kSwerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerveDrive::setModuleStates,
                swerveDrive);

        addCommands(
//            new InstantCommand(() -> swerveDrive.resetOdometry(exampleTrajectory.getInitialPose())),
//            swerveControllerCommand,
            new InstantCommand(() -> swerveDrive.resetOdometryAndAngleOffset(
                new Pose2d(
                    ballBlueLeft.getInitialState().poseMeters.getX(),
                    ballBlueLeft.getInitialState().poseMeters.getY(),
                    ballBlueLeft.getInitialState().holonomicRotation),
                false)),
            pathPlannerCommand,
            new InstantCommand(() -> swerveDrive.drive(0.0, 0.0, 0.0, true, false))       
        );
    }
}