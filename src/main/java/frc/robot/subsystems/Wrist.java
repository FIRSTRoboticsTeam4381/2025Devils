// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
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

  //create Wrist
  public Wrist() {
    wrist1 = new SparkFlex(56, MotorType.kBrushless);

    SparkFlexConfig wrist1Config = new SparkFlexConfig();
      wrist1Config.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

      wrist1Config.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(1)
      .i(0)
      .d(0);
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

    wrist1.getAbsoluteEncoder().getPosition();
  }


  public Command joystickCtrl(Supplier<Double> downJoystickValue, Supplier<Double> upJoystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> wrist1.set(upJoystickValue.get()-downJoystickValue.get()), this));
  }


  public Command wristPosition(double position) 
  {
    return new SparkPosition(wrist1, position, 0.05, this).withName("Wrist to" + position); // Will add positions later
  }


  // Level commands
  public Command l1() 
  {
    return wristPosition(0.5001).withName("Wrist Level 1"); // Will NEED to update positions (currently 0 as defualt)
  }
  public Command l2() 
  {
    return wristPosition(0.6271).withName("Wrist Level 2");
  }
  public Command l3() 
  {
    return wristPosition(0.879).withName("Wrist Level 3");
  }
  public Command l4() 
  {
    return wristPosition(0.879).withName("Wrist Level 4");
  }


  // Scoring commands
  public Command processorCommand() 
  {
    return wristPosition(7.3589).withName("Processor Scoring");
  }
  public Command bargeCommand() 
  {
    return wristPosition(0).withName("Barge Scoring");
  }


  // Pickup commands
  public Command groundPickupLeft() 
  {
    return wristPosition(0.6796).withName("Wrist Ground Pickup Left"); // Will need to change #s
  }

  /* 
  public Command groundPickupRight() 
  {
    return wristPosition(0.1709).withName("Wrist Ground Pickup Right"); // Will need to change #s
  }
    */

  public Command coralStation() 
  {
    return wristPosition(0.4029).withName("Wrist Coral Station");
  }

  public Command zero() 
  {
    return wristPosition(0.428).withName("Zero");
  }
  // WILL NEED TO PROGRAM TO MAKE THE WRIST PARALLEL WITH GROUND(?)UNTIL IT IS INTO THE FINAL POSITION TO SCORE

  private double arbFeedforward(){
    // [(Arm Weight) * (CG Length)] / [(Stall Torque) * (# of Motors) * (Gear Ratio)] * cos(theta)

    double weight = 15.0; // (lbs)
    double cgLength = 6.33; // (inches)
    double stallTorque = 3.6; // (Newton meters)
    int numMotors = 1;
    double gearRatio = 97.5;
    double angle = wrist1.getAbsoluteEncoder().getPosition(); // TODO the absolute encoder will need to be configured 0-360 with 0 straight up

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
