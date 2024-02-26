// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeLeader = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax intakeFollower = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax intakeSupport = new CANSparkMax(8, MotorType.kBrushless);
  private PIDController supportPidController = new PIDController(0.005, 0, 0);

  private RelativeEncoder supportEncoder = intakeSupport.getEncoder();
  double currentIntakeDegree = 0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeFollower.follow(intakeLeader);
    supportEncoder.setPosition(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentIntakeDegree = ((-supportEncoder.getPosition()/48)*360);
    SmartDashboard.putNumber("Intake Degree", currentIntakeDegree);
  }

  public void autoSupportUp(){
    intakeSupport.set(-supportPidController.calculate(currentIntakeDegree, -195)); 
  }

  public void autoSupportDown(){
    intakeSupport.set(-supportPidController.calculate(currentIntakeDegree, -7));
  }

  public void intake(){
    intakeLeader.set(0.4);
  }

  public void outtake(){
    intakeLeader.set(-0.4);
  }

  public void supportUp(){
    intakeSupport.set(0.7);
  }

  public void supportDown(){
    intakeSupport.set(-0.7);
  }

  public void stopIntakeMotors(){
    intakeLeader.set(0);
  }

  public void stopSupportMotor(){
    intakeSupport.set(0);
  }
}
