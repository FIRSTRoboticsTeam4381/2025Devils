// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged
public class Wrist extends SubsystemBase 
{
    public SparkFlex wrist1; // Currently assuming one motor on the wrist

    public AbsoluteEncoder position;

    public Supplier<Double> joystickValue;

    public double value;

  //create Wrist
  public Wrist() {
    wrist1 = new SparkFlex(56, MotorType.kBrushless);

    SparkFlexConfig wrist1Config = new SparkFlexConfig();
      wrist1Config.smartCurrentLimit(30).idleMode(IdleMode.kCoast);

    wrist1.configure(wrist1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    this.setDefaultCommand( // Stop motor I think
        new FunctionalCommand(() -> {
        },
            () -> wrist1.set(0),
            (killed) -> {
            },
            () -> {
              return false;
            },
            this));

    value = 0;
  }


  public Command joystickCtrl(Supplier<Double> downJoystickValue, Supplier<Double> upJoystickValue) 
  {
    if(downJoystickValue.get() > 0)
    {
      value = -(downJoystickValue.get());
    }
    if(upJoystickValue.get() > 0)
    {
      value = upJoystickValue.get();
    }

    return new RepeatCommand(
      new InstantCommand(() -> wrist1.set(value), this));
  }


  public Command wristPosition(double position) 
  {
    return new SparkPosition(wrist1, position, 1.0, this).withName("Wrist to" + position); // Will add positions later
  }


  // Level commands
  public Command l1() 
  {
    return wristPosition(0).withName("Wrist Level 1"); // Will NEED to update positions (currently 0 as defualt)
  }
  public Command l2() 
  {
    return wristPosition(0).withName("Wrist Level 2");
  }
  public Command l3() 
  {
    return wristPosition(0).withName("Wrist Level 3");
  }
  public Command l4() 
  {
    return wristPosition(0).withName("Wrist Level 4");
  }


  // Scoring commands
  public Command processorCommand() 
  {
    return wristPosition(0).withName("Processor Scoring");
  }
  public Command bargeCommand() 
  {
    return wristPosition(0).withName("Barge Scoring");
  }


  // Pickup commands
  public Command groundPickup() 
  {
    return wristPosition(0).withName("Wrist Ground Pickup"); // Will need to change #s
  }
  public Command coralStation() 
  {
    return wristPosition(0).withName("Wrist Coral Station");
  }

  // WILL NEED TO PROGRAM TO MAKE THE WRIST PARALLEL WITH GROUND(?)UNTIL IT IS INTO THE FINAL POSITION TO SCORE



  
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
