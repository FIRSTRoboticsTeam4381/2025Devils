// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;


@Logged
public class SwingArm extends SubsystemBase 
{
  /** Creates a new SwingArm. */

  public SparkFlex rotate1;
  public SparkFlex rotate2;
  
  public AbsoluteEncoder angle;

  private Extender extender;

  /** This is the ground intake **/
  public SwingArm(Extender extender)
  {
    rotate1 = new SparkFlex(53, MotorType.kBrushless);
    rotate2 = new SparkFlex(52, MotorType.kBrushless);
    
    angle = rotate1.getAbsoluteEncoder();
    this.extender=extender;
    

    SparkMaxConfig rotateConfig1 = new SparkMaxConfig();
    SparkMaxConfig rotateConfig2 = new SparkMaxConfig();

    rotateConfig1
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .absoluteEncoder.inverted(true);
    rotateConfig1.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(2.5)
      .i(0)
      .d(0)
    .outputRange(-.375, .375);
    rotateConfig2
      .apply(rotateConfig1)
      .follow(rotate1)
      .idleMode(IdleMode.kBrake);
    
      

    rotate1.configure(rotateConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rotate2.configure(rotateConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    /*
    this.setDefaultCommand(
      new FunctionalCommand(() -> {}, 
      () -> rotate.set(0), 
      (killed) -> {}, 
      () -> {return false;}, 
      this)
    );
  
    */

    rotate1.getEncoder().getVelocity();
    angle.getPosition();
  }

  public double getAngle()
  {
    return rotate1.getAbsoluteEncoder().getPosition() * (rotate1.configAccessor.absoluteEncoder.getPositionConversionFactor()==360.0 ? 1 : 360.0);
  }

  public Command goToAngle(double angle)
  {
    return new SparkPosition(rotate1, angle, .05, this).withName("Rotating to " + angle); // TODO arbitrary feedforward will need to be included
  }
  public void setPositionReference(double angle)
  {
    rotate1.getClosedLoopController().setReference(angle, ControlType.kPosition); // TODO arbitrary feedforward will need to be included
  }

  public Command l1L() 
  {
    return goToAngle(0.4).withName("Level 1 Left");
  }

  public Command l2L() 
  {
    return goToAngle(0.4228).withName("Level 2 Left");
  }

  public Command l3L() 
  {
    return goToAngle(0.24).withName("Level 3 Left");
  }

  public Command l4L() 
  {
    return goToAngle(0.0570).withName("Level 4 Left");
  }

  public Command l1R() 
  {
    return goToAngle(0.6).withName("Level 1 Right");
  }

  public Command l2R() 
  {
    return goToAngle(0.5772).withName("Level 2 Right");
  }

  public Command l3R() 
  {
    return goToAngle(0.76).withName("Level 3 Right");
  }

  public Command l4R() 
  {
    return goToAngle(0.943).withName("Level 4 Right");
  }


  public Command coralStationL() 
  {
    return goToAngle(0.388).withName("Coral Station Left");
  }
  public Command coralStationR() 
  {
    return goToAngle(0.64).withName("Coral Station Right");
  }

  public Command processorL()
  {
    return goToAngle(0).withName("Processor Left");
  }
  public Command processorR()
  {
    return goToAngle(0).withName("Processor Right");
  }

  public Command groundPickupLeft() {
    return goToAngle(0.428).withName("Ground Pickup Left Side");
  }

  public Command groundPickupRight() {
    return goToAngle(0.652).withName("Ground Pickup Right Side");
  }


  public Command swing(Supplier<Double> joystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> rotate1.set(joystickValue.get()), this));
  }
  public Command nothing() 
  {
    return new RepeatCommand(
      new InstantCommand(() -> rotate1.set(0), this));
  }

  public Command zero() 
  {
    return goToAngle(0.5).withName("Zero");
  }

  public double arbFeedforward(){
    // [(Arm Weight) * (CG Length)] / [(Stall Torque) * (# of Motors) * (Gear Ratio)] * cos(theta)

    // TODO you can combine the math into one line, I just leave it split up initially to make troubleshooting easier
    // TODO keep in mind direction may need to be flipped

    double weight = 33.0; // (lbs)
    double percentExtension = extender.getPosition() / extender.MAX_EXTENSION; // TODO
    double cgLength = 11.5 + (18.5 - 11.5) * percentExtension; // (inches)
    double stallTorque = 3.6; // (Newton meters)
    int numMotors = 2;
    double gearRatio = 100.0;
    double angle = rotate1.getAbsoluteEncoder().getPosition() *360.0;

    double inlbsTorque = weight*cgLength;
    double nmTorque = inlbsTorque*0.112985;

    return nmTorque / (stallTorque * numMotors * gearRatio) * Math.sin(angle); // This is sin because straight up is 0 degrees
  }

  

   @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm/Arbitrary Feedforward", arbFeedforward());

      SmartDashboard.putData(this);
  }
}
