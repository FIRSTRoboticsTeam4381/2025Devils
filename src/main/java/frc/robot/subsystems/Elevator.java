package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged
public class Elevator extends SubsystemBase {

    private SparkFlex motor1;


   // create Elevator
  public Elevator()
  {
    motor1 = new SparkFlex(50, MotorType.kBrushless);


    SparkMaxConfig motor1Config = new SparkMaxConfig();
      motor1Config
        .smartCurrentLimit(10)
        .idleMode(IdleMode.kCoast);

    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    this.setDefaultCommand(
      new FunctionalCommand(() -> {}, 
      () -> motor1.set(0), 
      (killed) -> {}, 
      () -> {return false;}, 
      this)
    );
  }


  @Override
   // This method will be called once per scheduler run
  public void periodic() 
  {
    SmartDashboard.putData(this);
  }

   // up/down using the joystick
  public Command elevate(double speed)
  {
    return new InstantCommand(() -> motor1.set(speed));
  }


   // preset position commands
  public Command l1(double distance)
  {
    return new SparkPosition(motor1, distance, 1.0, this);
  }

  public Command l2(double distance)
  {
    return new SparkPosition(motor1, distance, 1.0, this);
  }

  public Command l3(double distance)
  {
    return new SparkPosition(motor1, distance, 1.0, this);
  }

  public Command l4(double distance)
  {
    return new SparkPosition(motor1, distance, 1.0, this);
  }
}
