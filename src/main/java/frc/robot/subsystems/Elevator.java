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
public class Elevator extends SubsystemBase {

    private SparkFlex motor1;


   // create Elevator
  public Elevator() {
    motor1 = new SparkFlex(50, MotorType.kBrushless);


    SparkFlexConfig motor1Config = new SparkFlexConfig();
      motor1Config
        .smartCurrentLimit(10)
        .idleMode(IdleMode.kCoast);

    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
   
  public void periodic() {
  // This method will be called once per scheduler run

    SmartDashboard.putData(this);
  }

   // up/down using the joystick
  public Command elevatorJoystick(Supplier<Double> joystickValue) {
    return new RepeatCommand(
      new InstantCommand(() -> motor1.set(joystickValue.get()), this));
  }

  // GO TO command
  public Command elevatorTo(double distance) {
    return new SparkPosition(motor1, distance, 1.0, this);
  } 


  // preset position commands:
  public Command l1() {
    return elevatorTo(0).withName("Level 1"); // ALL DISTANCE VALUES will be determined after we get robot
  }

  public Command l2() {
    return elevatorTo(0).withName("Level 2");
  }

  public Command l3() {
    return elevatorTo(0).withName("Level 3");
  }

  public Command l4() {
    return elevatorTo(0).withName("Level 4");
  }
}
