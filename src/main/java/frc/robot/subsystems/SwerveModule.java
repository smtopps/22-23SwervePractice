// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder steerEncoder;
  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double steerEncoderOffset) {
    steerEncoder = new CANCoder(steerEncoderID);
    driveMotor = new TalonFX(driveMotorID);
    steerMotor = new TalonFX(steerMotorID);

    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 80, 0.0));
    steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0.0));
    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Brake);

    steerMotor.configSelectedFeedbackCoefficient((1.0/2048.0*360.0) * ((15.0/32.0)*(10.0/60.0)));
    steerMotor.configFeedbackNotContinuous(false, 20);
    
    steerMotor.config_kP(0, 0.2);
    steerMotor.config_kI(0, 0.0);
    steerMotor.config_kD(0, 0.1);
    steerMotor.config_kF(0, 0.0);

    driveMotor.setSelectedSensorPosition(0.0);
    steerMotor.setSelectedSensorPosition(steerEncoder.getAbsolutePosition() + steerEncoderOffset);
  }

  public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition();
  }

  public double getSteeringPosition() {
    return steerMotor.getSelectedSensorPosition();
  }

  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity();
  }

  public double getSteeringVelocity() {
    return steerMotor.getSelectedSensorVelocity();
  }

  public double getSteerEncoderRad() {
    return Units.degreesToRadians(steerEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteeringPosition()));
  }

  public void set(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    steerMotor.set(ControlMode.Position, state.angle.getRadians());
  }

  public void setState(double speed, double angle) {
    driveMotor.set(ControlMode.PercentOutput, 0.0);
    steerMotor.set(ControlMode.Position, angle);
  }

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0.0);
    steerMotor.set(ControlMode.PercentOutput, 0.0);
  }
}