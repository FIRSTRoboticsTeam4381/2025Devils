// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged
public class Hang extends SubsystemBase 
{
  public SparkFlex hang1;

  public AbsoluteEncoder position;


  // create Hang
  public Hang() 
  {
    hang1 = new SparkFlex(61, MotorType.kBrushless);

    SparkFlexConfig hang1Config = new SparkFlexConfig();
      hang1Config
        .smartCurrentLimit(10)
        .idleMode(IdleMode.kBrake);

    hang1.configure(hang1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public Command hangPosition(double position) 
  {
    return new SparkPosition(hang1, position, 0, null).withName("Hang to " + position);
  }

  // hanging
  public Command hang() 
  {
    return hangPosition(0).withName("Hang");
  }
  public Command unhang() 
  {
    return hangPosition(0).withName("Un Hang");
  }



  
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
