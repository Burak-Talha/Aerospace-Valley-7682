// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftLeader = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  // The motors on the right side of the drive.
  private final CANSparkMax m_rightLeader = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  // The left-side drive encoder
  private final RelativeEncoder m_lefRelativeEncoder = m_leftLeader.getEncoder();

  // The right-side drive encoder
  private final RelativeEncoder m_rightRelativeEncoder = m_rightLeader.getEncoder();

  // The gyro sensor
  private final AHRS navx = new AHRS();
  private double currentAngle = 0;

    // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  private final Field2d field2d = new Field2d();

  private final PIDController xPidController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
  private final PIDController yawPidController = new PIDController(AutoConstants.kPYawController, AutoConstants.KIYawController, AutoConstants.kDYawController);
  private final PIDController turnPidController = new PIDController(AutoConstants.kPTurnController, AutoConstants.KITurnController, AutoConstants.kDTurnController);

  private final XboxController xboxController = new XboxController(0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);
    navx.reset();
    resetEncoders();
    SmartDashboard.putData(field2d);
    m_odometry =
        new DifferentialDriveOdometry(
            navx.getRotation2d(), getLeftRelativeEncoderDistance(), getRightRelativeEncoderDistance());

          AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::drive,
                AutoConstants.kRamseteB,
                AutoConstants.kRamseteZeta, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        navx.getRotation2d(), getLeftRelativeEncoderDistance(), getRightRelativeEncoderDistance());
    field2d.setRobotPose(m_odometry.getPoseMeters());
    System.out.print("BLUE ANGLE:"+getAngleCalculationForBlue());
    System.out.println("RED ANGLE:"+getYaw());
  }

   public void drive(ChassisSpeeds speeds){
    m_drive.feed();  

    xPidController.setSetpoint(speeds.vxMetersPerSecond);
    yawPidController.setSetpoint(speeds.omegaRadiansPerSecond);

    m_drive.arcadeDrive(xPidController.calculate(getChassisSpeeds().vxMetersPerSecond), yawPidController.calculate(getChassisSpeeds().omegaRadiansPerSecond));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void i_drive(ChassisSpeeds speeds){
    m_drive.feed();  

    xPidController.setSetpoint(speeds.vxMetersPerSecond);
    yawPidController.setSetpoint(speeds.omegaRadiansPerSecond);

    m_drive.arcadeDrive(xPidController.calculate(getChassisSpeeds().vxMetersPerSecond), yawPidController.calculate(getChassisSpeeds().omegaRadiansPerSecond));
  }

    public void turnXdegrees(double targetSetpoint){
    try{
      currentAngle = getPose().getRotation().getDegrees();
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      currentAngle = getAngleCalculationForBlue();
    }
    }catch(Exception exception){
    }
      //getPose().getRotation().getDegrees()
      arcadeDrive((xboxController.getRawAxis(3)-xboxController.getRawAxis(2))*0.5, turnPidController.calculate(currentAngle, targetSetpoint)*0.6);
    }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_lefRelativeEncoder.setPosition(0);
    m_rightRelativeEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((m_lefRelativeEncoder.getPosition() + m_rightRelativeEncoder.getPosition()) / 2.0)*Constants.DriveConstants.WHEEL_CIRCUMFERENCE/Constants.DriveConstants.GEAR_RATIO;
  }
  public Command followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return new FollowPathRamsete(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getLeftRelativeEncoderDistance() {
    return (m_lefRelativeEncoder.getPosition()*DriveConstants.WHEEL_CIRCUMFERENCE)/DriveConstants.GEAR_RATIO;
  }

  public double getRightRelativeEncoderDistance() {
    return (m_rightRelativeEncoder.getPosition()*DriveConstants.WHEEL_CIRCUMFERENCE)/DriveConstants.GEAR_RATIO;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_lefRelativeEncoder.getVelocity(), m_rightRelativeEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        navx.getRotation2d(), getLeftRelativeEncoderDistance(), getRightRelativeEncoderDistance(), pose);
  }

  public void zeroYaw(){
    navx.setAngleAdjustment(0);
    navx.reset();
    navx.zeroYaw();
  }
  
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_lefRelativeEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightRelativeEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getYaw(){
    return navx.getYaw();
  }

  public double getAngleCalculationForBlue(){
    return (getHeading() + 360) % 360;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navx.getRate();
  }
}