// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DRIVER_CONTROLLER = 0;

    public static final int DRIVETRAIN_PIGEON_ID = 2; // Pigeon ID

    public static final double DRIVE_SPEED = 0.5;
    public static final double BOOST_SPEED = 1.0;
    public static final double PERCISION_SPEED = 0.25;

    public static final class SwerveConstants {
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(15.5);
        public static final double WHEELBASE_METERS = Units.inchesToMeters(17.5);

        public static final double MAX_VOLTAGE = 12.0;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)) * 0.10033 * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.TRACKWIDTH_METERS / 2.0, SwerveConstants.WHEELBASE_METERS / 2.0), // Front Left
            new Translation2d(SwerveConstants.TRACKWIDTH_METERS / 2.0, -SwerveConstants.WHEELBASE_METERS / 2.0), // Front Right
            new Translation2d(-SwerveConstants.TRACKWIDTH_METERS / 2.0, SwerveConstants.WHEELBASE_METERS / 2.0), // Back Left
            new Translation2d(-SwerveConstants.TRACKWIDTH_METERS / 2.0, -SwerveConstants.WHEELBASE_METERS / 2.0)); // Back Right

        public static final int FRONT_LEFT_DRIVE_MOTOR = 16; // Front left module drive motor ID
        public static final int FRONT_LEFT_STEER_MOTOR = 17; // Front left module steer motor ID
        public static final int FRONT_LEFT_STEER_ENCODER = 18; // Front left steer encoder ID
        public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(175.51); // Front left steer offset

        public static final int FRONT_RIGHT_DRIVE_MOTOR = 10; // Front right drive motor ID
        public static final int FRONT_RIGHT_STEER_MOTOR = 11; // Front right steer motor ID
        public static final int FRONT_RIGHT_STEER_ENCODER =12; // Front right steer encoder ID
        public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(94.83); // Front right steer offset

        public static final int BACK_LEFT_DRIVE_MOTOR = 19; // Back left drive motor ID
        public static final int BACK_LEFT_STEER_MOTOR = 20; // Back left steer motor ID
        public static final int BACK_LEFT_STEER_ENCODER = 21; // Back left steer encoder ID
        public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(219.11); // Back left steer offset

        public static final int BACK_RIGHT_DRIVE_MOTOR = 14; // Back right drive motor ID
        public static final int BACK_RIGHT_STEER_MOTOR = 13; // Back right steer motor ID
        public static final int BACK_RIGHT_STEER_ENCODER = 15; // Back right steer encoder ID
        public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(338.28); // Back right steer offset
    }

    public static final class AutoConstants {
        public static final double kPXController = 3.0; //1.5
        public static final double kPYController = 3.0; //1.5
        public static final double kPThetaController = 3.0;
    }
}
