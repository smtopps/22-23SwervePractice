// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Drive");

    frontLeftModule = new SwerveModule(
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR,
      SwerveConstants.FRONT_LEFT_STEER_MOTOR,
      SwerveConstants.FRONT_LEFT_STEER_ENCODER,
      SwerveConstants.FRONT_LEFT_STEER_OFFSET);

    frontRightModule = new SwerveModule(
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR,
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR,
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER,
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET);

    backLeftModule = new SwerveModule(
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR,
      SwerveConstants.BACK_LEFT_STEER_MOTOR,
      SwerveConstants.BACK_LEFT_STEER_ENCODER,
      SwerveConstants.BACK_LEFT_STEER_OFFSET);

    backRightModule = new SwerveModule(
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    frontLeftModule.set(states[0]);
    frontRightModule.set(states[1]);
    backLeftModule.set(states[2]);
    backRightModule.set(states[3]);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  public void lock() {
    frontLeftModule.setState(0, 45);
    frontRightModule.setState(0, -45);
    backLeftModule.setState(0, -45);
    backRightModule.setState(0, 45);
  }

  public SwerveModuleState getFLState() {
    return frontLeftModule.getState();
  }

  public SwerveModuleState getFRState() {
    return frontRightModule.getState();
  }

  public SwerveModuleState getBLState() {
    return backLeftModule.getState();
  }

  public SwerveModuleState getBRState() {
    return backRightModule.getState();
  }

  public PathPlannerTrajectory loadTrajectoryFromFile(String filename, double maxVel, double maxAccel) {
    try {
      return loadPathPlannerTrajectory(filename, maxVel, maxAccel);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return null;
    }
  }

  protected static PathPlannerTrajectory loadPathPlannerTrajectory(String trajectoryName, double maxVel, double maxAccel) throws IOException {
    return PathPlanner.loadPath(trajectoryName, maxVel, maxAccel);
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(AutoConstants.kPXController, 0, 0);
    yController = new PIDController(AutoConstants.kPYController, 0, 0);
    thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0); //Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,
      () -> PoseEstimator.getCurrentPose(),
      SwerveConstants.KINEMATICS,
      xController,
      yController,
      thetaController,
      this::setModuleStates,
      eventMap,
      this);
    return swerveControllerCommand.andThen(() -> stop());
  }

  public double getCurrentChassisSpeeds() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getFLState(), this.getFRState(), this.getBLState(), this.getBRState());
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }

  public Rotation2d getCurrentChassisHeading() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getFLState(), this.getFRState(), this.getBLState(), this.getBRState());
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(PoseEstimator.getCurrentPose().getRotation());
    return currentHeading;
  }
}