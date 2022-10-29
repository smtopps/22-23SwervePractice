// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PigeonSubsystemTwo;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithJoysticks2 extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PigeonSubsystemTwo pigeonSubsystemTwo;

  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks2(SwerveSubsystem swerveSubsystem, PigeonSubsystemTwo pigeonSubsystemTwo, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    this.swerveSubsystem = swerveSubsystem;
    this.pigeonSubsystemTwo = pigeonSubsystemTwo;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.xLimiter = new SlewRateLimiter(1.0);
    this.yLimiter = new SlewRateLimiter(1.0);
    this.turnLimiter = new SlewRateLimiter(1.0);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*swerveSubsystem.drive(
      new ChassisSpeeds(
        modifyAxis(translationY.getAsDouble(), 0.25, yLimiter) * swerveSubsystem.maxVelocityMetersPerSecond,
        modifyAxis(translationX.getAsDouble(), 0.25, xLimiter) * swerveSubsystem.maxVelocityMetersPerSecond,
        modifyAxis(rotation.getAsDouble(), 0.25, turnLimiter) * swerveSubsystem.maxAngularVelocityRadiansPerSecond));*/

    swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      modifyAxis(translationY.getAsDouble(), RobotContainer.maxSpeed, yLimiter) * swerveSubsystem.maxVelocityMetersPerSecond, 
      modifyAxis(translationX.getAsDouble(), RobotContainer.maxSpeed, xLimiter) * swerveSubsystem.maxVelocityMetersPerSecond, 
      modifyAxis(rotation.getAsDouble(), RobotContainer.maxSpeed, turnLimiter) * swerveSubsystem.maxAngularVelocityRadiansPerSecond, 
      pigeonSubsystemTwo.getGyroRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter) {
    value = MathUtil.applyDeadband(value, 0.02);
    value = Math.copySign(value * value, value);
    value = value*speedModifyer;
    value = limiter.calculate(value);
    return value;
  }
}
