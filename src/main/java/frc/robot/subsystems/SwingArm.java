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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;



public class SwingArm extends SubsystemBase 
{
  /** Creates a new SwingArm. */

  public SparkFlex rotate;
  
  public AbsoluteEncoder angle;

  private Extender extender;

  // The positions change them when we get them
  public static final double rotateL4 = 0;
  public static final double rotateL3 = 0;
  public static final double rotateL2 = 0;
  public static final double rotateL1 = 0;
  
  

  /** This is the ground intake **/
  public SwingArm(Extender extender)
  {
    rotate = new SparkFlex(52, MotorType.kBrushless);
    
    angle = rotate.getAbsoluteEncoder();

    this.extender=extender;
    

    SparkMaxConfig rotateConfig = new SparkMaxConfig();
    

    rotateConfig
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake);
    rotateConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(2.7)
      .i(0)
      .d(0);
      

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

    rotate.getEncoder().getVelocity();
    angle.getPosition();
  }

  public double getAngle()
  {
    return rotate.get();
  }

  public Command goToAngle(double angle)
  {
    return new SparkPosition(rotate, angle, .05, this).withName("Rotating to " + angle);
  }

  public Command l1() 
  {
    return goToAngle(0.4249).withName("Level 1");
  }

  public Command l2() 
  {
    return goToAngle(0.4228).withName("Level 2");
  }

  public Command l3() 
  {
    return goToAngle(0.1097).withName("Level 3");
  }

  public Command l4() 
  {
    return goToAngle(0.0570).withName("Level 4");
  }

  public Command coralStation() 
  {
    return goToAngle(0.5653).withName("Coral Station");
  }

  public Command processor()
  {
    return goToAngle(0.5226).withName("Processor");
  }

  public Command groundPickupLeft() {
    return goToAngle(-5.8951).withName("Ground Pickup Left Side");
  }

  public Command groundPickupRight() {
    return goToAngle(8.7227).withName("Ground Pickup Right Side");
  }


  public Command swing(Supplier<Double> joystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> rotate.set(joystickValue.get()), this));
  }

  public Command zero() 
  {
    return goToAngle(0.476).withName("Ground Pickup Right");
  }

  private double arbFeedforward(){
    // [(Arm Weight) * (CG Length)] / [(Stall Torque) * (# of Motors) * (Gear Ratio)] * cos(theta)

    double weight = 33.0; // (lbs)
    double percentExtension = extender.getPosition() / extender.MAX_EXTENSION;
    double cgLength = 11.5 + (18.5 - 11.5) * percentExtension; // (inches)
    double stallTorque = 3.6; // (Newton meters)
    int numMotors = 2;
    double gearRatio = 100.0;
    double angle = rotate.getAbsoluteEncoder().getPosition(); // TODO the absolute encoder will need to be configured 0-360 with 0 straight up

    double inlbsTorque = weight*cgLength;
    double nmTorque = inlbsTorque*0.112985;

    return nmTorque / (stallTorque * numMotors * gearRatio) * Math.cos(angle);
  }

  

   @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
