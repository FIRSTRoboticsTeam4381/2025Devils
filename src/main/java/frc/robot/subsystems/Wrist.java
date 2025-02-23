// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // TODO change to kPrimaryEncoder if the adjusted position works
      .p(3)
      .i(0)
      .d(0)
      .outputRange(-1, 1);
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

  public Command joystickCtrl(Supplier<Double> downJoystickValue, Supplier<Double> upJoystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> wrist1.set(upJoystickValue.get()-downJoystickValue.get()), this));
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
  public Command l1L() 
  {
    return wristPosition(0.7).withName("Level 1 Left"); // Will NEED to update positions (currently 0 as defualt)
  }
  public Command l2L() 
  {
    return wristPosition(0.64).withName("Level 2 Left");
  }
  public Command l3L() 
  {
    return wristPosition(0.516).withName("Level 3 Left");
  }
  public Command l4L() 
  {
    return wristPosition(0.153).withName("Level 4 Left");
  }

  public Command l1R() 
  {
    return wristPosition(0.0).withName("Level 1 Right"); // Will NEED to update positions (currently 0 as defualt)
  }
  public Command l2R() 
  {
    return wristPosition(0.0).withName("Level 2 Right");
  }
  public Command l3R() 
  {
    return wristPosition(0.0).withName("Level 3 Right");
  }
  public Command l4R() 
  {
    return wristPosition(0.0).withName("Level 4 Right");
  }


  public Command coralStationL() 
  {
    return wristPosition(0).withName("Wrist Coral Station Left");
  }
  public Command coralStationR() 
  {
    return wristPosition(0.802).withName("Wrist Coral Station Right");
  }
  
  public Command processorL() 
  {
    return wristPosition(0).withName("Processor Scoring Left");
  }
  public Command processorR() 
  {
    return wristPosition(0).withName("Processor Scoring Right");
  }

  public Command bargeCommand() 
  {
    return wristPosition(0).withName("Barge Scoring");
  }


  
  public Command groundPickupLeft() 
  {
    return wristPosition(0.551).withName("Wrist Ground Pickup Left"); // Will need to change #s
  }

  
  public Command groundPickupRight() 
  {
    return wristPosition(1.38).withName("Wrist Ground Pickup Right"); // Will need to change #s
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
