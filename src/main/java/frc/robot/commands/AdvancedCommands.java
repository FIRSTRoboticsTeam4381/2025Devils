// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  /**
   * A command that requires all specials subsystems, but does nothing with them
   * This only exists to prevent control returning to the joystick, and hold mechanisms in place
   * (If you want joystick control back, you will need a button to do so)
   * @return
   */
  public Command holdPositionCommand(){
    return new FunctionalCommand(
      ()->{}, ()->{}, (interrupted)->{}, ()->{return false;}, 
      robot.wrist,robot.extender,robot.swingArm,robot.elevator);
  }

  /**
   * A wrapper command that will be used with likely every position command on the robot.
   * Every position command in here should be wrapped with this to pull the arm in first,
   * and hold everything in place afterwards.
   * I made this a seperate wrapper command so we don't have to rewrite everything to
   * be a sequential annoyingly
   * @param positions The command to run on the robot. Basically just insert the command group you 
   * already made as the argument.
   * @return The command group given as the argument wrapped with a zeroing command first, and a holding command after.
   */
  public Command combinedPositionCommand(Command positions){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(robot.wrist.zero(), robot.extender.zero()),
      positions,
      holdPositionCommand()
    );
  }
  
  public Command l1L()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l1L(),
        robot.swingArm.l1L(),
        robot.wrist.l1L()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.l1L()
    ))); 
    
  }
  public Command l2L()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l2L(),
        robot.swingArm.l2L(),
        robot.extender.l2L(),
        robot.wrist.l2L()
      ));
    
  }
  public Command l3L()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l3L(),
        robot.swingArm.l3L(),
        robot.wrist.l3L(),
        robot.extender.l3L()
        
      ));
    
  }
  public Command l4L()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.wrist.l4L(),
        robot.swingArm.l4L()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.l4L(),
        robot.elevator.l4L()
        
      )));
  }


  public Command l1R()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l1R(),
        robot.swingArm.l1R()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.l1R(),
        robot.wrist.l1R()
    ))); 
    
  }
  public Command l2R()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l2R(),
        robot.swingArm.l2R()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.l2R(),
        robot.wrist.l2R()
    )));
    
  }
  public Command l3R()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l3R(),
        robot.swingArm.l3R()
        
      ).andThen(new ParallelCommandGroup(
        //robot.extender.l3R(),
        robot.wrist.l3R()
      )));
    
  }
  public Command l4R()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l4R(),
        robot.swingArm.l4R()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.l4R(),
        robot.wrist.l4R()
      )));
  }



  public Command coralStationR()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.coralStationR(),
        robot.swingArm.coralStationR()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.coralStationR(),
        robot.wrist.coralStationR()
      ).andThen(new ParallelCommandGroup(
        robot.intake.coralInOrOutR()
      ).andThen(
        robot.aCommands.zeroEverything()
      )
      )));
  }

  public Command coralStationL()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.coralStationL(),
        robot.swingArm.coralStationL()
        
      ).andThen(new ParallelCommandGroup(
        robot.extender.coralStationL(),
        robot.wrist.coralStationL()
      ).andThen(new ParallelCommandGroup(
        robot.intake.coralInOrOutL()
      ).andThen(
        robot.aCommands.zeroEverything()
      )
      )));
  }


  public Command processorL()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.processorL(),
        robot.swingArm.processorL()

      ).andThen(new ParallelCommandGroup(
        robot.extender.processorL(),
        robot.wrist.processorL()
    )));
  }
  public Command processorR()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.processorR(),
        robot.swingArm.processorR()

      ).andThen(new ParallelCommandGroup(
        robot.extender.processorR(),
        robot.wrist.processorR()
    )));
  }


  public Command groundPickupLeft()
  { 
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.groundPickupLeft(),
        robot.swingArm.groundPickupLeft()

      ).andThen(new ParallelCommandGroup(
        robot.extender.groundPickupLeft(),
        robot.wrist.groundPickupLeft()
    )));
  }

  
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
    

  public Command zeroEverything()
  { 
    return new ParallelCommandGroup
    (
      robot.elevator.elevatorTo(10)
    ).andThen(new ParallelCommandGroup(
      robot.extender.zero(),
      robot.wrist.zero()
    )).andThen(new ParallelCommandGroup(
      robot.elevator.zero(),
      robot.swingArm.zero()
    ));
  }
  
  
}
