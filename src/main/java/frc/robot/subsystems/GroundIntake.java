// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class GroundIntake extends SubsystemBase {
  
  public SparkMax motor1;
  public SparkMax motor2;

  /** This is the ground intake **/
  public GroundIntake() 
  {
    motor1 = new SparkMax(61, MotorType.kBrushless);
    motor2 = new SparkMax(62, MotorType.kBrushless);

    SparkMaxConfig motor1Config = new SparkMaxConfig();

    motor1Config
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kCoast);
    
    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    SparkMaxConfig motor2Config = new SparkMaxConfig();
    motor2Config.apply(motor1Config);
    motor2Config.follow(motor1, true);

    motor2.configure(motor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    this.setDefaultCommand(
      new FunctionalCommand(() -> {}, 
      () -> motor1.set(0), 
      (killed) -> {}, 
      () -> {return false;}, 
      this)
    );
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putData(this);
  }

  // Bring in
  public Command intake()
  {
    return new InstantCommand(() -> motor1.set(.7)).withName("Ground Intaking");
  }

  // Shoot out if stuck
  public Command outtake()
  {
    return new InstantCommand(() -> motor1.set(.7)).withName("Ground Outtaking");
  }
}
