// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;



public class SwingArm extends SubsystemBase {
  /** Creates a new SwingArm. */
  
  public SparkFlex rotate;
  public SparkMax extend;
  public AbsoluteEncoder angle;

  // The positions change them when we get them
  public static final double rotateL4 = 0;
  public static final double rotateL3 = 0;
  public static final double rotateL2 = 0;
  public static final double rotateL1 = 0;
  
  public static final double extendL4 = 0;
  public static final double extendL3 = 0;
  public static final double extendL2 = 0;
  public static final double extendL1 = 0;


  /** This is the ground intake **/
  public SwingArm()
  {
    rotate = new SparkFlex(52, MotorType.kBrushless);
    extend = new SparkMax(53, MotorType.kBrushless);
    

    SparkMaxConfig rotateConfig = new SparkMaxConfig();
    SparkMaxConfig extendConfig = new SparkMaxConfig();

    rotateConfig
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kCoast);
    
    rotate.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    extendConfig
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kCoast);
    
    extend.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    this.setDefaultCommand(
      new FunctionalCommand(() -> {}, 
      () -> rotate.set(0), 
      (killed) -> {}, 
      () -> {return false;}, 
      this)
    );
  }

  public double getAngle()
  {
    return rotate.get();
  }

  public Command goToAngle(double angle)
  {
    return new SparkPosition(rotate, angle, 1.0, this).withName("Rotating to " + angle);
  }

  public double getExtension()
  {
    return extend.get();
  }

  public Command goToDistance(double distance)
  {
    return new SparkPosition(extend, distance, 1.0, this).withName("Going to "+ distance);
  }

}
