// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

public class Extender extends SubsystemBase 
{
  /** Creates a new Extender. */

  public SparkMax extend;

  public static final double extendL4 = 0;
  public static final double extendL3 = 0;
  public static final double extendL2 = 0;
  public static final double extendL1 = 0;

  public Extender() 
  {
    extend = new SparkMax(53, MotorType.kBrushless);

    SparkMaxConfig extendConfig = new SparkMaxConfig();

    extendConfig
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kCoast);
    
    extend.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getExtension()
  {
    return extend.get();
  }

  public Command goToDistance(double distance)
  {
    return new SparkPosition(extend, distance, 1.0, this).withName("Going to "+ distance);
  }

  public Command extend(Supplier<Double> joystickValue)
  {
    return new RepeatCommand(
      new InstantCommand(() -> extend.set(joystickValue.get()), this));
  }

  // GO TO command
  public Command extendTo(double distance) {
    return new SparkPosition(extend, distance, 1.0, this);
  } 

  
   // preset reef position commands:
   public Command l1() {
    return extendTo(0).withName("Level 1");
  }

  public Command l2() {
    return extendTo(0).withName("Level 2");
  }

  public Command l3() {
    return extendTo(0).withName("Level 3");
  }

  public Command l4() {
    return extendTo(0).withName("Level 4");
  }

  public Command barge() {
    return extendTo(0).withName("Barge");
  }

  public Command groundPickup() {
    return extendTo(0).withName("Ground Pickup");
  }

  public Command sourcePickup() {
    return extendTo(0).withName("Source Pickup");
  }

  public Command processor() {
    return extendTo(0).withName("Processor");
  }
}
