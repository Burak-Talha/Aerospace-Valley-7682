// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Commands.OuttakeCmd;
import frc.robot.Commands.ResetOdometerCommand;
import frc.robot.Commands.ShootXrpmCommand;
import frc.robot.Commands.SupportDownCommand;
import frc.robot.Commands.SupportUpCommand;
import frc.robot.Commands.TurnToTarget;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_robotShooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    (m_driverController.getRawAxis(3)-m_driverController.getRawAxis(2)*0.7), -m_driverController.getLeftX()*0.6),
            m_robotDrive));
    // Defult Intake Mode
    m_intakeSubsystem.setDefaultCommand(new SupportDownCommand(m_intakeSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Shoot Commands
    new JoystickButton(m_driverController, Button.kA.value).whileTrue(new TurnToTarget(m_robotDrive).repeatedly());
    new JoystickButton(m_driverController, Button.kX.value).whileTrue(new ShootXrpmCommand(m_robotShooter, 1000).repeatedly());
    // Automatic intake/out Commands
    new JoystickButton(m_driverController, Button.kY.value).whileTrue(new ParallelCommandGroup(new SequentialCommandGroup(new WaitCommand(1.5), new OuttakeCmd(m_intakeSubsystem).repeatedly()), new SupportUpCommand(m_intakeSubsystem).repeatedly()));
    new JoystickButton(m_driverController, Button.kB.value).whileTrue(new ParallelCommandGroup(new SequentialCommandGroup(new WaitCommand(1.5), new IntakeCmd(m_intakeSubsystem).repeatedly()), new SupportUpCommand(m_intakeSubsystem).repeatedly()));
    // Manual support up/down commands
    new JoystickButton(m_driverController, Button.kLeftBumper.value).whileTrue(new SupportUpCommand(m_intakeSubsystem).repeatedly());
    new JoystickButton(m_driverController, Button.kRightBumper.value).whileTrue(new SupportUpCommand(m_intakeSubsystem).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ResetOdometerCommand(m_robotDrive);
  }
}
