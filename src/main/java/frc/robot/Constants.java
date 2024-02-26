// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 4;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 5;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double WHEEL_CIRCUMFERENCE = 0.4787787204;
    public static final double GEAR_RATIO = 10.71; 

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class ShooterConstants{
    public static double SHOOTER_LEFT = 7;
    public static double SHOOTER_RIGHT = 6;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2.3;
    public static final double kRamseteZeta = 1.5;
    // X-vector Controller
    public static final double kPXController = 0.8;
    public static double kIXController = 0.0001;
    public static double kDXController = 0.00001;
    // Yaw Controller
    public static final double kPYawController = 0.8;
    public static final double KIYawController = 0.0001;
    public static double kDYawController = 0.00001;

    // TUrn Controller
    public static final double kPTurnController = 0.04;
    public static final double KITurnController = 0.0009;
    public static double kDTurnController = 0.0000000;
  }

    public static final class FieldConstants{
    public static final Pose2d BLUE_SUB_WOOFER = new Pose2d(new Translation2d(0, 5.60), new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_SUB_WOOFER = new Pose2d(new Translation2d(16.5, 5.60), new Rotation2d(Math.toRadians(0)));
  }
}