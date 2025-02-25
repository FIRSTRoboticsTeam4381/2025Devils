package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged
public class Elevator extends SubsystemBase
{
    private SparkFlex elevator1;
    private SparkFlex elevator2;

  // create Elevator
  public Elevator() 
  {
    elevator1 = new SparkFlex(50, MotorType.kBrushless);
    elevator2 = new SparkFlex(51, MotorType.kBrushless);
    SparkFlexConfig elevator1Config = new SparkFlexConfig();
      elevator1Config
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake)
        .limitSwitch.forwardLimitSwitchEnabled(true).reverseLimitSwitchEnabled(true);
    
    SparkFlexConfig elevator2Config = new SparkFlexConfig();
      elevator2Config
        .apply(elevator1Config)
        .follow(elevator1,true)
        .limitSwitch.forwardLimitSwitchEnabled(true).reverseLimitSwitchEnabled(true);

    elevator1Config.closedLoop
      .p(0.07)
      .i(0)
      .d(0);
    
    elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevator1.getEncoder().getPosition();
  }


  // up/down joystickCtrl
  public Command joystickCtrl(Supplier<Double> joystickValue) 
  {
    return new RepeatCommand(
      new InstantCommand(() -> elevator1.set(-joystickValue.get()), this));
  }
  public Command nothing() 
  {
    return new RepeatCommand(
      new InstantCommand(() -> elevator1.set(0), this));
  }

  // GO TO  command
  public Command elevatorTo(double distance) 
  {
    return new SparkPosition(elevator1, distance, 1.0, this);
  } 


  // preset position commands:
  public Command l1L() 
  {
    return elevatorTo(0).withName("Level 1 Left");
  }
  public Command l2L() 
  {
    return elevatorTo(30).withName("Level 2 Left");
  }
  public Command l3L() 
  {
    return elevatorTo(14.5).withName("Level 3 Left");
  }
  public Command l4L() 
  {
    return elevatorTo(37.5).withName("Level 4 Left");
  }

  public Command l1R() 
  {
    return l1L().withName("Level 1 Right");
  }
  public Command l2R() 
  {
    return l2L().withName("Level 2 Right");
  }
  public Command l3R() 
  {
    return l3L().withName("Level 3 Right");
  }
  public Command l4R() 
  {
    return l4L().withName("Level 4 Right");
  }
  

  public Command coralStationL()
  {
    return coralStationR().withName("Coral Station Left");
  }
  public Command coralStationR()
  {
    return elevatorTo(30).withName("Coral Station Right");
  }
  
  
  public Command processorL() {
    return elevatorTo(0).withName("Processor");
  }
  public Command processorR() {
    return elevatorTo(0).withName("Processor");
  }

  public Command groundPickupLeft() {
    return elevatorTo(0).withName("Ground Pickup Left");
  }

  public Command groundPickupRight() {
    return elevatorTo(0).withName("Ground Pickup Right");
  }
  public Command zero() {
    return elevatorTo(0).withName("Zeroing");
  }

  @Override 
  public void periodic() 
  {
    // This method will be called once per scheduler run
      SmartDashboard.putData(this);
  }
}
