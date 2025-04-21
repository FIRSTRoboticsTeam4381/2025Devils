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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AdvancedCommands 
{
    RobotContainer robot;
    boolean pro;

  //public Supplier<Boolean> algaeBoolean = robot.armIntake.algaeSensor::get;

  public AdvancedCommands(RobotContainer r)
  {
    robot = r;
    pro = false;
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
      //new ParallelCommandGroup(robot.wrist.zero(), robot.extender.zero()),
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
        robot.wrist.zero(),
        robot.elevator.l4(),
        robot.extender.l4()
      ).andThen(new ParallelCommandGroup(
        robot.wrist.l4(),
        robot.swingArm.l4()
      )));
  }

  public Command l4A()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l4(),
        robot.extender.l4(),
        robot.wrist.l4(),
        robot.swingArm.l4()
      ));
  }

  public Command algael2()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l2(),
        robot.extender.l2(),
        robot.swingArm.algael2(),
        robot.wrist.algael2()
      ));
  }
 
  public Command algael3()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.elevator.l3(),
        robot.extender.l3()
      ).andThen(new ParallelCommandGroup(
        robot.wrist.algael3(),
        robot.swingArm.algael3()
      )));
  }


  public Command coralStation()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.swingArm.coralStation(),
        robot.wrist.coralStation(),
        robot.elevator.coralStation(),
        robot.extender.coralStation()
      ).andThen(new ParallelCommandGroup(
        robot.intake.coralInOrOut()
      ).andThen(
        robot.aCommands.zeroEverything()
      )
      )
    );
  }

  public Command coralStationL1()
  {
    return combinedPositionCommand(
      new ParallelCommandGroup(
        robot.swingArm.coralStationL1(),
        robot.wrist.coralStationL1(),
        robot.elevator.zero(),
        robot.extender.zero()
      ).andThen(new ParallelCommandGroup(
        robot.intake.intakeL1Coral()
      ).andThen(
        robot.aCommands.zeroEverything()
      )
      )
    );
  }


  public Command processor()
  {
    return combinedPositionCommand(
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new RepeatCommand(new InstantCommand(()->robot.intake.intake1.set(0.5))),
          new ParallelCommandGroup(
            robot.wrist.preprocessor()
        )),
        new ParallelRaceGroup(
          new RepeatCommand(new InstantCommand(()->robot.intake.intake1.set(0.5))),
          new ParallelCommandGroup(
          robot.elevator.processor(),
          robot.extender.processor(),
          robot.swingArm.processor()
          )
        ),
        new ParallelRaceGroup(
          robot.intake.holdAlgae(),
          new ParallelCommandGroup(
          robot.wrist.processor()
        )),
        new ParallelCommandGroup(
          robot.intake.holdAlgae()
        )
      )
    );
  }

  public Command hang()
  {
    return combinedPositionCommand(
        new ParallelCommandGroup(
        robot.elevator.hang(),
        robot.extender.hang(),
        robot.swingArm.hang(),
        robot.wrist.hang()
        )
    );
  }

  /* 
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
    */
  public Command groundPickup()
  { 
    return combinedPositionCommand(
      new SequentialCommandGroup(
      new ParallelCommandGroup(
        robot.elevator.groundPickup(),
        robot.extender.groundPickup(),
        robot.wrist.groundPickup()
      ),
      new ParallelCommandGroup(
        robot.swingArm.groundPickup()
       )
      )
    );
  }
  public Command barge()
  { 
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        robot.intake.holdAlgae(),
      new ParallelCommandGroup(
        robot.elevator.barge(),
        robot.extender.barge(),
        robot.swingArm.barge(),
        robot.wrist.algaeHold()
        
      )),
      new ParallelCommandGroup(
        robot.wrist.barge(),
        robot.intake.holdAlgae()
      )
    );
  }
  public Command bargeR()
  { 
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        robot.intake.holdAlgae(),
      new ParallelCommandGroup(
        robot.elevator.barge(),
        robot.extender.barge(),
        robot.swingArm.barge(),
        robot.wrist.algaeHold()
        
      )),
      new ParallelCommandGroup(
        robot.wrist.bargeR(),
        robot.intake.holdAlgae()
      )
    );
  }
    
  public Command algaeHolding()
  { 
    return new SequentialCommandGroup(
    robot.intake.intakeAlgae(),
    new ParallelRaceGroup(
      robot.intake.holdAlgae(),
      new ParallelCommandGroup(
        robot.extender.algaeHold(),
        new ConditionalCommand(new WaitCommand(0), robot.swingArm.algaeHold(), () ->robot.swingArm.angle.getPosition() < 0.3)
       )
      ),
      new ParallelCommandGroup(
        new ConditionalCommand(
          new SequentialCommandGroup(
            new ParallelRaceGroup(
              robot.intake.holdAlgae(),
              new ParallelCommandGroup(
              robot.extender.zero(),
              robot.elevator.processor()
              )
            ),
            new ParallelRaceGroup(
              robot.intake.holdAlgae(),
              new ParallelCommandGroup(
                robot.swingArm.processor(),
                robot.wrist.processor()
              )
            )
          )
          ,  
        new ParallelCommandGroup(
          new ParallelRaceGroup(
              robot.intake.holdAlgae(),
              new ParallelCommandGroup(
                robot.extender.zero(),
                robot.elevator.algaeHold(),
                robot.wrist.algaeHold(),
                robot.swingArm.zero()
              )
            )
        )
          , 
        () -> robot.swingArm.curAng < 0.4)  
      ),
    robot.intake.holdAlgae()
    );
  }

  public Command algaeInOrOut() 
  {
    return new ConditionalCommand(robot.intake.ejectAlgae(), algaeHolding(), () -> {return robot.intake.hasAlgae;}).withName("Algae In or Out");
  }
  

  public Command zeroEverything()
  { 
    return new ParallelCommandGroup
    (
      robot.swingArm.zero()
    ).andThen(new ParallelCommandGroup(
      robot.wrist.zero()
    )).andThen(new ParallelCommandGroup(
      robot.extender.zero(),
      robot.elevator.zero()
    ));
  }
  public Command proZeroEverything()
  { 
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            robot.intake.holdAlgae(),
      new ParallelCommandGroup
      (
        robot.wrist.preprocessor()
      ).andThen(
        robot.swingArm.zero()
      ).andThen(new ParallelCommandGroup(
        robot.wrist.algaeHold()
      )).andThen(new ParallelCommandGroup(
        robot.extender.zero(),
        robot.elevator.zero()
      ))),
      robot.intake.holdAlgae())
      ;
  }
  
}
