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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;
import frc.robot.commands.SparkPositionProfiled;


@Logged
public class SwingArm extends SubsystemBase 
{
  /** Creates a new SwingArm. */

  public SparkFlex rotate1;
  public SparkFlex rotate2;
  
  public AbsoluteEncoder angle;
  public double curAng;

  private Extender extender;
  
  public SparkMaxConfig rotateConfig1;
  public SparkMaxConfig rotateConfig2;
  public SparkMaxConfig rotateConfig3;
  public SparkMaxConfig rotateConfig4;

  public double lastTarget = 0;

  /** This is the ground intake **/
  public SwingArm(Extender extender)
  {
    rotate1 = new SparkFlex(52, MotorType.kBrushless);
    rotate2 = new SparkFlex(53, MotorType.kBrushless);
    
    angle = rotate1.getAbsoluteEncoder();
    

    this.extender=extender;
    

    rotateConfig1 = new SparkMaxConfig();
    rotateConfig2 = new SparkMaxConfig();
    rotateConfig3 = new SparkMaxConfig();
    rotateConfig4 = new SparkMaxConfig();

    rotateConfig1
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .absoluteEncoder.inverted(true);
    rotateConfig1.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(2.5)
      .i(0)
      .d(0)
      .outputRange(-.3, .3);
    rotateConfig2
      .apply(rotateConfig1)
      .follow(rotate1)
      .idleMode(IdleMode.kBrake);


    rotateConfig3
      .smartCurrentLimit(80)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .absoluteEncoder.inverted(true);
    rotateConfig3.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(2.5)
      .i(0)
      .d(0)
      .outputRange(-.1, .1);
    rotateConfig4
      .apply(rotateConfig3)
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
    rotate2.getEncoder().getVelocity();
    angle.getPosition();
  }

  public double getAngle()
  {
    return rotate1.getAbsoluteEncoder().getPosition() * (rotate1.configAccessor.absoluteEncoder.getPositionConversionFactor()==360.0 ? 1 : 360.0);
  }

  public Command goToAngle(double angle)
  {
    return new SparkPosition(rotate1, angle, .02, this).alongWith(new InstantCommand(() -> lastTarget = angle)).withName("Rotating to " + angle); // TODO arbitrary feedforward will need to be included
  }
  public void setPositionReference(double angle)
  {
    rotate1.getClosedLoopController().setReference(angle, ControlType.kPosition); // TODO arbitrary feedforward will need to be included
  }

  public Command l1() 
  {
    return goToAngle(0.55).withName("Level 1");
  }

  public Command l2() 
  {
    return goToAngle(0.535).withName("Level 2");
  }

  public Command l3() 
  {
    return goToAngle(0.535).withName("Level 3");
  }

  public Command l4() 
  {
    return goToAngle(0.532).withName("Level 4");
  }

  public Command algael2() 
  {
    return goToAngle(0.531).withName("Algae Level 2");
  }

  public Command algael3() 
  {
    return goToAngle(0.531).withName("Algae Level 3");
  }


  public Command coralStation() 
  {
    return goToAngle(0.365).withName("Coral Station");
  }
  public Command coralStationL1() 
  {
    return goToAngle(0.49).withName("Coral Station L1");
  }

  public Command hang() {
    return goToAngle(0.72).withName("Hang");
  }

  public Command processor()
  {
    return goToAngle(0.125).withName("Processor");
  }

  public Command groundPickup() {
    return goToAngle(0.255).withName("Ground Pickup");
  }

  public Command barge() {
    return goToAngle(.5).withName("Barge");
  }

  public Command algaeHold() {
    return goToAngle(.45).withName("Algae Hold");
  }

  public Command swing(Supplier<Double> joystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> rotate1.set(-joystickValue.get()), this));
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

  // Main Configs Below
  public Command armConfig1()
  {
    return new InstantCommand(() ->
        rotate1.configure(rotateConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)      
    );
  }
  public Command armConfig2()
  {
    return new InstantCommand(() ->
        rotate2.configure(rotateConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)      
    );
  }

  // Hang Configs Below
  public Command armConfig3()
  {
    return new InstantCommand(() ->
        rotate1.configure(rotateConfig3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)      
    );
  }
  public Command armConfig4()
  {
    return new InstantCommand(() ->
        rotate2.configure(rotateConfig4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)      
    );
  }

  public double arbFeedforward(){
    // [(Arm Weight) * (CG Length)] / [(Stall Torque) * (# of Motors) * (Gear Ratio)] * cos(theta)

    // TODO you can combine the math into one line, I just leave it split up initially to make troubleshooting easier
    // TODO keep in mind direction may need to be flipped

    double weight = 33.0; // (lbs)
    double percentExtension = 0; // TODO
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
      curAng = angle.getPosition();
  }
}
