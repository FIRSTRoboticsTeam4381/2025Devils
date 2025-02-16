// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

public class Extender extends SubsystemBase 
{
    public SparkFlex extend1;

    public static final double extendL4 = 0;
    public static final double extendL3 = 0;
    public static final double extendL2 = 0;
    public static final double extendL1 = 0;

  // create Extender
  public Extender() 
  {
    extend1 = new SparkFlex(53, MotorType.kBrushless);

    SparkFlexConfig extend1Config = new SparkFlexConfig();

    extend1Config
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .limitSwitch.forwardLimitSwitchEnabled(true).reverseLimitSwitchEnabled(true);
    extend1Config.closedLoop
      .p(2.7)
      .i(0)
      .d(0);
    extend1.configure(extend1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    extend1.getEncoder().getPosition();
  }


  public double getExtension()
  {
    return extend1.get();
  }


  //GO TO command
  public Command extenderTo(double distance)
  {
    return new SparkPosition(extend1, distance, 1.0, this).withName("Going to "+ distance);
  }


  public Command extend(Supplier<Double> joystickValue)
  {
    return new RepeatCommand(
      new InstantCommand(() -> extend1.set(joystickValue.get()), this));
  }


  // GO TO command
  public Command extendTo(double distance) 
  {
    return new SparkPosition(extend1, distance, 1.0, this);
  } 


  // preset reef position commands:
  public Command l1()
  {
    return extendTo(0).withName("Level 1");
  }
  public Command l2()
  {
    return extendTo(0).withName("Level 2");
  }
  public Command l3()
  {
    return extendTo(23.57).withName("Level 3");
  }
  public Command l4()
  {
    return extendTo(23.57).withName("Level 4");
  }


  public Command barge() 
  {
    return extendTo(0).withName("Barge");
  }


  /*public Command groundPickup()
  {
    return extendTo(0).withName("Ground Pickup");
  }*/


  public Command coralStation()
  {
    return extendTo(0).withName("Coral Station");
  }


  public Command processor()
  {
    return extendTo(14.0405).withName("Processor");
  }


  public Command groundPickupLeft()
 {
    return extendTo(13.2475).withName("Ground Pickup Left");
 }  

 public Command groundPickupRight() 
{
  return extendTo(12.8804).withName("Ground Pickup Right");
}
  public Command zero() 
  {
    return extendTo(0).withName("Zero");
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
