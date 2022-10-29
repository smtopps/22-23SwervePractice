// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  private final double maxVoltage = 12.0;

  public final double maxVelocityMetersPerSecond = 6380.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  public final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond / Math.hypot(SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0 , SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //Front Left
    new Translation2d(SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //Front Right
    new Translation2d(SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //Back Left
    new Translation2d(-SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //Back Right
    new Translation2d(-SwerveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -SwerveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private PIDController xController;
  private PIDController yController;
  private ProfiledPIDController thetaController;

  private final Field2d field2d = new Field2d();

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, 
    PigeonSubsystemTwo.getGyroscopeRotation(), 
    new Pose2d(0, 0, new Rotation2d(0)));

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Drive");

    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
      Mk4SwerveModuleHelper.GearRatio.L2,
      SwerveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
      SwerveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
      SwerveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
      SwerveConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0), 
      Mk4SwerveModuleHelper.GearRatio.L2, 
      SwerveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0), 
      Mk4SwerveModuleHelper.GearRatio.L2,
      SwerveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
      SwerveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
      SwerveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
      SwerveConstants.BACK_LEFT_MODULE_STEER_OFFSET);

    backRightModule = Mk4SwerveModuleHelper.createFalcon500(
      swerveTab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0), 
      Mk4SwerveModuleHelper.GearRatio.L2, 
      SwerveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_MODULE_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_MODULE_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  @Override
  public void periodic() {
    odometry.update(PigeonSubsystemTwo.getGyroscopeRotation(), getFLState(), getFRState(), getBLState(), getBRState()); //path planning
    field2d.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field", field2d);
    // This method will be called once per scheduler run
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocityMetersPerSecond);
    frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / maxVelocityMetersPerSecond * maxVoltage, states[3].angle.getRadians());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public void stop() {
    frontLeftModule.set(0, 0);
    frontRightModule.set(0, 0);
    backLeftModule.set(0, 0);
    backRightModule.set(0, 0);
  }

  public SwerveModuleState getFLState() {
    return new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle()));
  }

  public SwerveModuleState getFRState() {
    return new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle()));
  }

  public SwerveModuleState getBLState() {
    return new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle()));
  }

  public SwerveModuleState getBRState() {
    return new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()));
  }

  public Pose2d getPose() { // path planning
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose2d, double rotation) { // path planning
    //Pigeon2Subsystem.setGyroscopeRotation(rotation);
    odometry.resetPosition(pose2d, PigeonSubsystemTwo.getGyroscopeRotation());
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

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, Boolean initPose) {
    xController = new PIDController(AutoConstants.kPXController, 0, 0);
    yController = new PIDController(AutoConstants.kPYController, 0, 0);
    thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints); //Kp value, Ki=0, Kd=0, constraints value
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,
      this::getPose,
      kinematics,
      xController,
      yController,
      thetaController,
      this::setModuleStates,
      this
    );
    return swerveControllerCommand.andThen(() -> stop());
  }

  public void setOdometry(PathPlannerTrajectory trajectory) {
    PathPlannerState initialState = trajectory.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    resetOdometry(startingPose, initialState.holonomicRotation.getDegrees());
  }

  public Rotation2d getOdometryRotation2d() {
    return odometry.getPoseMeters().getRotation();
  }
}
