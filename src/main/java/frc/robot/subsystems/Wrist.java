// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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

    public ClosedLoopConfig wristCLC;

    public double value;

    private double initialPos;

  //create Wrist
  public Wrist() {
    wrist1 = new SparkFlex(56, MotorType.kBrushless);

    SparkFlexConfig wrist1Config = new SparkFlexConfig();
      wrist1Config
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake)
      .absoluteEncoder.positionConversionFactor(1.5625).inverted(true);

      wrist1Config.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)

      .p(20, ClosedLoopSlot.kSlot0)
      .i(0, ClosedLoopSlot.kSlot0)
      .d(2, ClosedLoopSlot.kSlot0)

      .p(15, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(2, ClosedLoopSlot.kSlot1)

      .outputRange(-1,1);

    wrist1.configure(wrist1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    
    
    
    

    initialPos = wrist1.getAbsoluteEncoder().getPosition();
    initialPos = wrist1.getEncoder().getPosition();


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

    wrist1.getAbsoluteEncoder().getPosition();
  }

  public Command joystickCtrl(Supplier<Double> JoystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> wrist1.set(JoystickValue.get()), this));
  }
  public Command nothing() 
  {
    return new RepeatCommand(
      new InstantCommand(() -> wrist1.set(0), this));
  }

 

  public Command wristPosition(double position) 
  {
    return new SparkPosition(wrist1, position, 0.05, this).withName("Wrist to" + position); // Will add positions later // TODO arbitrary feedforward will need to be included
  }
  public void setPositionReference(double angle)
  {
    wrist1.getClosedLoopController().setReference(angle, ControlType.kPosition); // TODO arbitrary feedforward will need to be included
  }


  // Level commands
  public Command l1() 
  {
    return wristPosition(0.78).withName("Level 1"); 
  }
  public Command l2() 
  {
    return wristPosition(0.689).withName("Level 2");
  }
  public Command l3() 
  {
    return wristPosition(0.689).withName("Level 3");
  }
  public Command l4() 
  {
    return wristPosition(0.6684).withName("Level 4");
  }

  public Command algael2() 
  {
    return wristPosition(0.757).withName("Algae Level 2");
  }
  public Command algael3() 
  {
    return wristPosition(0.757).withName("Algae Level 3");
  }

  public Command coralStation() 
  {
    return wristPosition(0.568).withName("Wrist Coral Station");
  }
  public Command coralStationL1() 
  {
    return wristPosition(1.322).withName("Wrist Coral Station L1");
  }
  
  public Command hang() {
    return wristPosition(1.23).withName("Hang");
  }

  public Command processor() 
  {
    return wristPosition(0.926).withName("Processor Scoring");
  }
  public Command preprocessor() 
  {
    return wristPosition(0.65).withName("Preprocessor");
  }

  public Command bargeCommand() 
  {
    return wristPosition(0).withName("Barge Scoring");
  }
  
  public Command algaeHold() {
    return wristPosition(1.034).withName("Algae Hold");
  }


  
  public Command groundPickup() 
  {
    return wristPosition(1.18).withName("Wrist Ground Pickup"); // Will need to change #s
  }
  public Command barge() 
  {
    return wristPosition(0.9).withName("Barge"); // Will need to change #s
  }

  public Command bargeR() 
  {
    return wristPosition(1.09).withName("BargeR"); // Will need to change #s
  }
  

  public Command zero() 
  {
    return wristPosition(0.78125).withName("Zero");
  }
  // WILL NEED TO PROGRAM TO MAKE THE WRIST PARALLEL WITH GROUND(?)UNTIL IT IS INTO THE FINAL POSITION TO SCORE

  public double arbFeedforward(){
    // [(Arm Weight) * (CG Length)] / [(Stall Torque) * (# of Motors) * (Gear Ratio)] * cos(theta)

    // TODO you can combine the math into one line, I just leave it split up initially to make troubleshooting easier
    // TODO keep in mind direction may need to be flipped

    double weight = 15.0; // (lbs)
    double cgLength = 6.33; // (inches)
    double stallTorque = 3.6; // (Newton meters)
    int numMotors = 1;
    double gearRatio = 97.5;
    double angle = wrist1.getAbsoluteEncoder().getPosition() *360.0;

    double inlbsTorque = weight*cgLength;
    double nmTorque = inlbsTorque*0.112985;

    return nmTorque / (stallTorque * numMotors * gearRatio) * Math.sin(angle); // This is sin because straight up is 180 degrees
  }

  /**
   * Get the position of the wrist from the primary encoder, adjusted for the gear ratio and offset
   * by the initial position of the wrist informed by the absolute encoder
   * <p>
   * In theory, if the wrist can do 1 full rotation in either direction this should return a value between -360 – 360 of whatever the vertical position is
   * @return The adjusted position of the wrist in <STRONG> degrees </STRONG>
   */
  public double getAdjustedPosition(){
    // TODO not sure if you will have to invert stuff for this to work (either inverting the motor or the abs encoder, we will have to test and see)
    // Motor to wrist gear ratio: 97.5

    // TODO you can combine the math into one line, I just leave it split up initially to make troubleshooting easier

    double wristRevolutions = wrist1.getEncoder().getPosition() / 97.5; // Motor revolves 97.5 times for every 1 mechanism revolution
    double wristDegrees = wristRevolutions * 360.0; // NOTE: This is wrist degrees MOVED, because the offset has not been applied yet
    double offsetPosition = wristDegrees + initialPos*360.0; // TODO either configure the absolute encoder conversion factor to 360 or multiply initialPos by 360

    return offsetPosition;
  }

  /**
   * <STRONG> WARNING: Do not use until {@link #getAdjustedPosition()} is tested and validated!! </STRONG>
   * <p>
   * Provides the necessary conversion from the positions you have been using to the offset value needed for the wrist motor.
   * Right now, the *360 is included, so if you switch over to using degrees you'll need to remove this.
   * <p>
   * Meant to integrate into existing code in the least invasive way possible
   * @param position The absolute position to run the wrist to. Should be able to keep using your same numbers
   * @return The adjusted position to feed directly into the wrist's PID control
   */
  public double adjustPosition(double position){
    // TODO you can combine the math into one line, I just leave it split up initially to make troubleshooting easier

    position=position*360.0;
    double offsetPosition = position-initialPos*360;
    double wristRevolutions = offsetPosition / 360.0;
    double motorRevolutions = wristRevolutions * 97.5;
    return motorRevolutions;
  }


  
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("wrist/Arbitrary Feedforward", arbFeedforward());
    SmartDashboard.putNumber("wrist/Absolute Encoder Degrees", wrist1.getAbsoluteEncoder().getPosition()*360.0);
    SmartDashboard.putNumber("wrist/Adjusted Primary Encoder Position", getAdjustedPosition());
    SmartDashboard.putNumber("wrist/Initial Position", initialPos);
    
    SmartDashboard.putData(this);
  }
}
