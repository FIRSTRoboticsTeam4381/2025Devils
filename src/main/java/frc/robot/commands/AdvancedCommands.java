// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    ).andThen(new ParallelCommandGroup(
      robot.armIntake.algaeInOrOut()
    )));
    
  }
  public Command l4()
  {
    return new ParallelCommandGroup(
      robot.elevator.l4(),
      robot.swingArm.l4()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.l4(),
      robot.wrist.l4()
    ).andThen(new ParallelCommandGroup(
      robot.armIntake.algaeInOrOut()
    )));
  }


  public Command coralStation()
  {
    return new ParallelCommandGroup(
      robot.elevator.coralStation(),
      robot.swingArm.coralStation()
      
    ).andThen(new ParallelCommandGroup(
      robot.extender.coralStation(),
      robot.wrist.coralStation()
    ).andThen(new ParallelCommandGroup(
      robot.armIntake.coralInOrOut()
    )));
  }


  public Command processor()
  { 
    return new ParallelCommandGroup(
      robot.elevator.processor(),
      robot.swingArm.processor()

    ).andThen(new ParallelCommandGroup(
      robot.extender.processor(),
      robot.wrist.processorCommand()
    ));
  }


  public Command groundPickupLeft()
  { 
    return new ParallelCommandGroup(
      robot.elevator.groundPickupLeft(),
      robot.swingArm.groundPickupLeft()

    ).andThen(new ParallelCommandGroup(
      robot.extender.groundPickupLeft(),
      robot.wrist.groundPickupLeft()
    ));
  }

  /* 
  public Command groundPickupRight()
  { 
    return new ParallelCommandGroup(
      robot.elevator.groundPickupRight(),
      robot.swingArm.groundPickupRight()

    ).andThen(new ParallelCommandGroup(
      robot.extender.groundPickupRight(),
      robot.wrist.groundPickupRight()
    ));
  }
    */

  public Command zeroEverything()
  { 
    return new ParallelCommandGroup(
      robot.elevator.elevatorTo(42),
      robot.extender.zero(),
      robot.wrist.zero()

    ).andThen(new ParallelCommandGroup(
      robot.elevator.zero(),
      robot.swingArm.zero()
    ));
  }
  
  
}
