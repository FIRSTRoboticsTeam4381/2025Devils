// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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
      .p(5)
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
    return wristPosition(0.7914).withName("Wrist Level 1"); // Will NEED to update positions (currently 0 as defualt)
  }
  public Command l2() 
  {
    return wristPosition(0.8461).withName("Wrist Level 2");
  }
  public Command l3() 
  {
    return wristPosition(0.1922).withName("Wrist Level 3");
  }
  public Command l4() 
  {
    return wristPosition(0.1922).withName("Wrist Level 4");
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
    return wristPosition(25.4721).withName("Wrist Ground Pickup Left"); // Will need to change #s
  }

  public Command groundPickupRight() 
  {
    return wristPosition(-48.1623).withName("Wrist Ground Pickup Right"); // Will need to change #s
  }

  public Command coralStation() 
  {
    return wristPosition(0.6215).withName("Wrist Coral Station");
  }

  public Command zero() 
  {
    return wristPosition(0.647).withName("Ground Pickup Right");
  }
  // WILL NEED TO PROGRAM TO MAKE THE WRIST PARALLEL WITH GROUND(?)UNTIL IT IS INTO THE FINAL POSITION TO SCORE



  
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
