// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimator;

public class SetPose extends CommandBase {
  private final PoseEstimator poseEstimator;
  private final Pose2d pose2d;
  /** Creates a new SetPose. */
  public SetPose(PoseEstimator poseEstimator, Pose2d pose2d) {
    this.poseEstimator = poseEstimator;
    this.pose2d = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poseEstimator.setPose(pose2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}