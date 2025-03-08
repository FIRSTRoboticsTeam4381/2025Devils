// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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
  


  public Command l1()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l1(),
        robot.extender.l1(),
        robot.swingArm.l1(),
        robot.wrist.l1()
      )); 
  }

  public Command l2()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l2(),
        robot.extender.l2(),
        robot.swingArm.l2(),
        robot.wrist.l2()
      ));
  }
 
  public Command l3()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l3(),
        robot.extender.l3()
      ).andThen(new ParallelCommandGroup(
        robot.wrist.l3(),
        robot.swingArm.l3()
      )));
  }
  

  public Command l4()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l4(),
        robot.extender.l4()
      ).andThen(new ParallelCommandGroup(
        robot.wrist.l4(),
        robot.swingArm.l4()
      )));
  }



  public Command coralStation()
  {
    return new ParallelCommandGroup(
        robot.swingArm.coralStation(),
        robot.wrist.coralStation()
      ).andThen(new ParallelCommandGroup(
        robot.intake.coralInOrOut()
      ).andThen(
        robot.aCommands.zeroEverything()
      )
      );
  }


  public Command processor()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.processor(),
        robot.extender.processor(),
        robot.swingArm.processor(),
        robot.wrist.processor()
    ));
  }


  public Command groundPickup()
  { 
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.groundPickup(),
        robot.extender.groundPickup(),
        robot.swingArm.groundPickup(),
        robot.wrist.groundPickup()
    ));
  }
  public Command barge()
  { 
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.barge(),
        robot.extender.barge(),
        robot.swingArm.barge(),
        robot.wrist.barge()
    ));
  }
    

  public Command zeroEverything()
  { 
    return new ParallelCommandGroup
    (
    ).andThen(new ParallelCommandGroup(
      robot.wrist.zero(),
      robot.swingArm.zero()
    )).andThen(new ParallelCommandGroup(
      robot.extender.zero(),
      robot.elevator.zero()
    ));
  }
  
  
}
