// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToLoadingStation extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;

  private final Pose2d finalPose2d = new Pose2d(Units.feetToMeters(51.0), Units.feetToMeters(13.5), new Rotation2d(Units.degreesToRadians(0.0)));
  private PathPlannerTrajectory trajectory;

  /** Creates a new DriveToLoadingStation. */
  public DriveToLoadingStation(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = PathPlanner.generatePath(
      new PathConstraints(3, 1),
      new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
      new PathPoint(new Translation2d(finalPose2d.getX()-0.5, finalPose2d.getY()), finalPose2d.getRotation(), finalPose2d.getRotation()),
      new PathPoint(new Translation2d(finalPose2d.getX(), finalPose2d.getY()), finalPose2d.getRotation(), finalPose2d.getRotation()));
      poseEstimator.setTrajectoryField2d(trajectory);
      swerveSubsystem.createCommandForTrajectory(trajectory).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return true;
  }
}