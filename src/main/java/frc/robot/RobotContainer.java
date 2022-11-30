// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Auto.TestPath;
import frc.robot.commands.ChangeMaxSpeed;
import frc.robot.commands.DriveToLoadingStation;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.LockDrive;
import frc.robot.commands.ToggleFieldRelative;
import frc.robot.commands.SetPose;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PhotonCamera photonCamera = new PhotonCamera("gloworm");
  public static double maxSpeed = Constants.DRIVE_SPEED;
  public static boolean fieldRelative = true;
  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final PoseEstimator poseEstimator = new PoseEstimator(photonCamera, swerveSubsystem, pigeon2Subsystem);

  //Auto Stuff
  private final TestPath testPath = new TestPath(swerveSubsystem, poseEstimator);
  SendableChooser<Command> chooser = new SendableChooser<>();

  //On The Fly Trajectory Stuff
  public static PathPlannerTrajectory trajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
      swerveSubsystem,
      poseEstimator,
      () -> -driverController.getLeftX(),
      () -> -driverController.getLeftY(),
      () -> -driverController.getRightX(),
      () -> fieldRelative,
      () -> maxSpeed));
    // Configure the button bindings
    configureButtonBindings();

    chooser.setDefaultOption("Triangle Path", testPath);
    SmartDashboard.putData(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, 8).whenPressed(new SetPose(poseEstimator, new Pose2d(0.0, 0.0, new Rotation2d(0.0))));
    new JoystickButton(driverController, 6).whenHeld(new ChangeMaxSpeed(Constants.BOOST_SPEED));
    new JoystickButton(driverController, 5).whenHeld(new ChangeMaxSpeed(Constants.PERCISION_SPEED));
    new JoystickButton(driverController, 3).whenPressed(new ToggleFieldRelative());
    new Button(driverController::getAButton).whenHeld(new LockDrive(swerveSubsystem));
    new JoystickButton(driverController, 4)
      .whenActive(new DriveToLoadingStation(swerveSubsystem, poseEstimator))
      .whenInactive(new InstantCommand(() -> {
        if(swerveSubsystem.getCurrentCommand() != null){
          swerveSubsystem.getCurrentCommand().cancel();
        }
      }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }
}