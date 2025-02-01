// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AdvancedCommands 
{
  RobotContainer robot;

  //public Supplier<Boolean> algaeBoolean = robot.armIntake.algaeSensor::get;

  public AdvancedCommands(RobotContainer r)
  {
    robot = r;
  }
  
  public Command l1()
  {
    return new ParallelCommandGroup(
      robot.elevator.l1(),
      robot.swingArm.l1()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.l1(),
      robot.wrist.l1()
    ));
    
  }
  public Command l2()
  {
    return new ParallelCommandGroup(
      robot.elevator.l2(),
      robot.swingArm.l2()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.l2(),
      robot.wrist.l2()
    ));
    
  }
  public Command l3()
  {
    return new ParallelCommandGroup(
      robot.elevator.l3(),
      robot.swingArm.l3()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.l3(),
      robot.wrist.l3()
    ));
    
  }
  public Command l4()
  {
    return new ParallelCommandGroup(
      robot.elevator.l4(),
      robot.swingArm.l4()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.l4(),
      robot.wrist.l4()
    ));
  }
  public Command coralStation()
  {
    return new ParallelCommandGroup(
      robot.elevator.coralStation(),
      robot.swingArm.coralStation()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.coralStation(),
      robot.wrist.coralStation()
    ));
  }

  public Command intakeAlgae() {
    return new RepeatCommand(
      robot.armIntake.algaeIntake()
    ).until(
      () -> robot.armIntake.algaeSensor.get()
    ).andThen(
      robot.armIntake.algaeStop()
    );
  }

  public Command intakeCoral() {
    return new RepeatCommand(
      robot.armIntake.coralIntake()
    ).until(
      () -> robot.armIntake.coralSensor.get()
    ).andThen(
      robot.armIntake.coralStop()
    );
  }

  public Command ejectAlgae() {
    return new RepeatCommand(
      robot.armIntake.algaeEject()
    ).until(
      () -> !robot.armIntake.algaeSensor.get() 
    ).andThen(
      robot.armIntake.algaeStop()
    );
  }

  public Command ejectCoral() {
    return new RepeatCommand(
      robot.armIntake.coralEject()
    ).until(
      () -> !robot.armIntake.coralSensor.get() 
    ).andThen(
      robot.armIntake.coralStop()
    );
  }

  public Command coralInOrOut() {
    return new ConditionalCommand(ejectCoral(), intakeCoral(), robot.armIntake.coralSensor::get);
  }

  public Command algaeInOrOut() {
    return new ConditionalCommand(ejectAlgae(), intakeAlgae(), robot.armIntake.algaeSensor::get);
  }
  
}
