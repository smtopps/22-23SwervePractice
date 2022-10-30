// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToLoadingStationGroup extends SequentialCommandGroup {
  private final Pose2d finalPose2d = new Pose2d(Units.feetToMeters(51.0), Units.feetToMeters(13.5), new Rotation2d(Units.degreesToRadians(0.0)));
  /** Creates a new DriveToLoadingStationGroup. */
  public DriveToLoadingStationGroup(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(3, 1,
      new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
      new PathPoint(new Translation2d(finalPose2d.getX()-0.5, finalPose2d.getY()), finalPose2d.getRotation(), finalPose2d.getRotation()),
      new PathPoint(new Translation2d(finalPose2d.getX(), finalPose2d.getY()), finalPose2d.getRotation(), finalPose2d.getRotation()));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetTrajectoryField2d(poseEstimator, trajectory),
      swerveSubsystem.createCommandForTrajectory(trajectory)
    );
  }
}