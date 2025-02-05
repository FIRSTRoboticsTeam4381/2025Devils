// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;



public class SwingArm extends SubsystemBase 
{
  /** Creates a new SwingArm. */
  
  public SparkFlex rotate;
  
  public AbsoluteEncoder angle;

  // The positions change them when we get them
  public static final double rotateL4 = 0;
  public static final double rotateL3 = 0;
  public static final double rotateL2 = 0;
  public static final double rotateL1 = 0;
  
  

  /** This is the ground intake **/
  public SwingArm()
  {
    rotate = new SparkFlex(52, MotorType.kBrushless);
    
    angle = rotate.getAbsoluteEncoder();
    

    SparkMaxConfig rotateConfig = new SparkMaxConfig();
    

    rotateConfig
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kCoast);
    rotateConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      

    rotate.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    /*
    this.setDefaultCommand(
      new FunctionalCommand(() -> {}, 
      () -> rotate.set(0), 
      (killed) -> {}, 
      () -> {return false;}, 
      this)
    );
    */
  }

  public double getAngle()
  {
    return rotate.get();
  }

  public Command goToAngle(double angle)
  {
    return new SparkPosition(rotate, angle, 1.0, this).withName("Rotating to " + angle);
  }

  public Command l1() 
  {
    return goToAngle(0).withName("Level 1");
  }

  public Command l2() 
  {
    return goToAngle(0).withName("Level 2");
  }

  public Command l3() 
  {
    return goToAngle(0).withName("Level 3");
  }

  public Command l4() 
  {
    return goToAngle(0).withName("Level 4");
  }

  public Command coralStation() 
  {
    return goToAngle(0).withName("Coral Station");
  }

  public Command swing(Supplier<Double> joystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> rotate.set(joystickValue.get()), this));
  }

  


   @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
