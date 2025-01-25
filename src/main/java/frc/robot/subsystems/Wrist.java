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

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
 
  public SparkMax wristMotor1; // Currently assuming one motor on the wrist
  
  public Wrist() {
    wristMotor1 = new SparkMax(56, MotorType.kBrushless);

    SparkMaxConfig motor1Config = new SparkMaxConfig();

    motor1Config.smartCurrentLimit(30).idleMode(IdleMode.kCoast);

    wristMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.setDefaultCommand( // Stop motor I think
      new FunctionalCommand(() -> {},
        () -> wristMotor1.set(0),
        (killed) -> {},
        () -> {return false;},
        this)
    );

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
