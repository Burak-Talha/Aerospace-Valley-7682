// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooter1 = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax shooter2 = new CANSparkMax(11, MotorType.kBrushless);

  private SparkMaxPIDController shooter1PIDController = shooter1.getPIDController();
  private SparkMaxPIDController shooter2PIDController = shooter2.getPIDController();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left shooter rpm:", shooter1.getEncoder().getVelocity());
    SmartDashboard.putNumber("right shooter rpm:", shooter2.getEncoder().getVelocity());
  }

  public void shoot(double rpm){
    shooter1PIDController.setReference(rpm, ControlType.kVelocity);
    shooter2PIDController.setReference(rpm, ControlType.kVelocity);
    //shooter1.set(-0.7);
    //shooter2.set(-0.7);
  }
  public void stopMotors(){
    shooter1.set(0);
    shooter2.set(0);
  }
}
