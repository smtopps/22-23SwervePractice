// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChangeMaxSpeed;
import frc.robot.commands.DriveWithJoysticks2;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.PigeonSubsystemTwo;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PigeonSubsystemTwo pigeonSubsystemTwo = new PigeonSubsystemTwo();

  public static double maxSpeed = 0.375;

  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks2(
      swerveSubsystem, 
      pigeonSubsystemTwo,
      () -> -driverController.getLeftX(), 
      () -> -driverController.getLeftY(),
      () -> -driverController.getRightX()));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, 8).whenPressed(new ZeroGyro(pigeonSubsystemTwo));
    new JoystickButton(driverController, 6).whenHeld(new ChangeMaxSpeed(0.75));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
