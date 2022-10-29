// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PigeonSubsystemTwo;

public class ZeroGyro extends CommandBase {
  private final PigeonSubsystemTwo pigeonSubsystemTwo;
  /** Creates a new ZeroGyro. */
  public ZeroGyro(PigeonSubsystemTwo pigeonSubsystemTwo) {
    this.pigeonSubsystemTwo = pigeonSubsystemTwo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pigeonSubsystemTwo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pigeonSubsystemTwo.zeroGyroscope();
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
