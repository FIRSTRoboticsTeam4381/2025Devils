// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AdvancedCommands;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonCam;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.HangCam;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwingArm;
import frc.robot.subsystems.Wrist;

@Logged
public class RobotContainer 
{
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController specialist = new CommandXboxController(1);

  //Auto Chooser
  SendableChooser<Autos.PreviewAuto> autoChooser = new SendableChooser<>();
  SendableChooser<Autos.PreviewAuto> autoBuilderChooser = new SendableChooser<>(); // Dont believe we use this

  // Subsystems
  public final Swerve swerve = new Swerve();
  //public final GroundIntake groundIntake = new GroundIntake();
  public final Intake intake = new Intake();
  public final Extender extender = new Extender();
  public final SwingArm swingArm = new SwingArm(extender);
  public final Wrist wrist = new Wrist();
  public final Elevator elevator = new Elevator();
  public final Hang hang = new Hang();
  public final AdvancedCommands aCommands;
  

  // camA, Left-Below (Forward), Translation(6.44733 in. LEFT, 8.26353 in. BACK, 10.62979 in. UP) - Rotation(60 deg. LEFT, 15 deg. UP, 0 deg.)
  public final PhotonCam camA = new PhotonCam("Camera_Left_Below", 
    new Transform3d(new Translation3d(Units.inchesToMeters(-8.26353), Units.inchesToMeters(6.44733), Units.inchesToMeters(10.62979)), 
    new Rotation3d(0*Math.PI/180.0, -15*Math.PI/180.0, 50*Math.PI/180.0)));

  // camB, Left-Upper (Backwards), Translation(6.36902 in. LEFT, 8.46948 in. BACK, 14.12979 in. UP) - Rotation(140 deg. LEFT, 15 deg. UP, 0 deg.)
  public final PhotonCam camB = new PhotonCam("Camera_Left_Upper", 
    new Transform3d(new Translation3d(Units.inchesToMeters(-8.46948), Units.inchesToMeters(6.36902), Units.inchesToMeters(14.12979)), 
    new Rotation3d(0*Math.PI/180.0, -15*Math.PI/180.0, 150*Math.PI/180.0)));

  // camC, Right-Below (Forwards), Translation(6.24207 in. RIGHT, 8.01891 in. BACK, 10.62979 in. UP) - Rotation(60 deg. RIGHT, 15 deg. UP, 0 deg.)
  public final PhotonCam camC = new PhotonCam("Camera_Right_Below", 
    new Transform3d(new Translation3d(Units.inchesToMeters(-8.01891), Units.inchesToMeters(-6.24207), Units.inchesToMeters(10.62979)), 
    new Rotation3d(0*Math.PI/180.0, -15*Math.PI/180.0, -50*Math.PI/180.0)));

  // camD, Right-Upper (Backwards), Translation(6.50299 in. RIGHT, 8.39213 in. BACK, 14.12979 in. UP) - Rotation(140 deg. RIGHT, 15 deg. UP, 0 deg.)
  public final PhotonCam camD = new PhotonCam("Camera_Right_Upper", 
    new Transform3d(new Translation3d(Units.inchesToMeters(-8.39213), Units.inchesToMeters(-6.50299), Units.inchesToMeters(14.12979)), 
    new Rotation3d(0*Math.PI/180.0, -15*Math.PI/180.0, -150*Math.PI/180.0)));

  public HangCam hangCam = new HangCam("HangCamera");

  // Constructor: set up the robot! 
  public RobotContainer() 
  {
    robotReference = this;

    aCommands = new AdvancedCommands(robotReference);


    // Add auto controls to the dashboard
    SmartDashboard.putData("Choose Auto:", autoChooser);
    //SmartDashboard.putData("Choose Reef Position:", autoBuilderChooser); // nevermind
    SmartDashboard.putString("Choose Reef Branch", ""); // When changed also change the key in Autos.java

    SmartDashboard.putString("Choose Level:", "");
    SmartDashboard.putData(CommandScheduler.getInstance());
    autoChooser.onChange((listener) -> listener.showPreview());
    SmartDashboard.putNumber("Start Delay",0);
    NamedCommands.registerCommand("Coral", intake.coralInOrOut());
    NamedCommands.registerCommand("Algae Intake/Outtake On", intake.algaeInOrOut());
    NamedCommands.registerCommand("Coral Station", aCommands.coralStation());
    
    NamedCommands.registerCommand("L1", aCommands.l1());
    NamedCommands.registerCommand("L2", aCommands.l2());
    NamedCommands.registerCommand("L3", aCommands.l3());
    NamedCommands.registerCommand("L4", aCommands.l4());
    

    NamedCommands.registerCommand("Level", aCommands.l4()); // Auto Chooser Level // Currently L4L because level is not fully functioning

    NamedCommands.registerCommand("Zero", aCommands.zeroEverything());



    // Set up autonomous picker
    // Add any autos you want to be able to select below
    // NOTE: This needs to be executed AFTER all NamedCommands have been registered!
    autoChooser.setDefaultOption("None", Autos.none());
    autoChooser.addOption("proSide E,B,A (trippple)", Autos.ProSideTrippple());
    autoChooser.addOption("antiSide J,A,B (trippple)", Autos.AntiSideTrippple());
    autoChooser.addOption("proSide D,C", Autos.ProSide_D_C());
    autoChooser.addOption("middleSide G,H", Autos.MiddleSide_G_H());
    autoChooser.addOption("proSide Basic (30s)", Autos.ProSideBasic());
    autoChooser.addOption("antiSide Basic (31s)", Autos.AntiSideBasic());
    autoChooser.addOption("Out The Way Blue", Autos.OutTheWayBlue());
    autoChooser.addOption("Out The Way Red", Autos.OutTheWayRed());
    autoChooser.addOption("ReefChooser: Processor Side Start", Autos.proSide());
    autoChooser.addOption("ReefChooser: Anti Processor Side Start", Autos.antiSide());
    autoChooser.addOption("ReefChooser: Middle Start", Autos.middle());

    
    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() 
  {
    driver.start()
      .onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
             true,
            driver.y()::getAsBoolean,
            driver.b()::getAsBoolean));

      specialist.a().onTrue(aCommands.groundPickup());
      specialist.b().onTrue(aCommands.coralStation());
      specialist.y().toggleOnTrue(intake.algaeInOrOut());
      specialist.leftBumper().onTrue(aCommands.barge());
      specialist.rightBumper().onTrue(aCommands.processor());
      
      specialist.povUp().onTrue(aCommands.l4());
      specialist.povLeft().onTrue(aCommands.l3());
      specialist.povRight().onTrue(aCommands.l2());
      specialist.povDown().onTrue(aCommands.l1());

      
      specialist.leftStick().or(specialist.rightStick()).onTrue(aCommands.zeroEverything());

      //elevator joystick controls
      elevator.setDefaultCommand(elevator.nothing());

      //wrist triggers  
      wrist.setDefaultCommand(wrist.nothing());

      //swing joystick controls
      swingArm.setDefaultCommand(swingArm.nothing());

      //extend joystick
      extender.setDefaultCommand(extender.nothing());

      //hang triggers  
      hang.setDefaultCommand(hang.joystickCtrl(interpolateJoystick(driver :: getLeftTriggerAxis, Constants.stickDeadband), interpolateJoystick(driver :: getRightTriggerAxis, Constants.stickDeadband)));
      
      specialist.back().or(driver.back()).onTrue(new InstantCommand(()->CommandScheduler.getInstance().cancelAll()));

      specialist.start().toggleOnTrue(new ParallelCommandGroup(
        elevator.joystickCtrl(interpolateJoystick(specialist:: getLeftY, Constants.stickDeadband)),
        wrist.joystickCtrl(interpolateJoystick(specialist :: getLeftTriggerAxis, Constants.stickDeadband), interpolateJoystick(specialist :: getRightTriggerAxis, Constants.stickDeadband)),
        swingArm.swing(interpolateJoystick(specialist :: getRightX, Constants.stickDeadband)),
        extender.extend(interpolateJoystick(specialist :: getRightY, Constants.stickDeadband))
        ));

      driver.rightBumper().toggleOnTrue(intake.coralInOrOut());
      driver.y().onTrue(swerve.highSpeedCommand());

      driver.a().whileTrue(new AutoAlign(swerve));
    }

  public Command getAutonomousCommand() 
  {
    Autos.pickPosition();
    double startDelay=SmartDashboard.getNumber("Start Delay", 0);


    return new SequentialCommandGroup( 
      new WaitCommand(startDelay), 
      new ScheduleCommand(autoChooser.getSelected().auto)); 
  }


  /**
   * Smooths joystic input for easier precice control without sacrificing full power.
   * @param in Input from joystic
   * @param deadzone Joystick deadzone
   * @return Transformed output
   */
  public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
  {
      return () -> interpolateNow(in.get(), deadzone);
  }

  public static double interpolateNow(double in, double deadzone)
  {
      if(Math.abs(in) < deadzone)
          return 0.0;
      else if (in>0)
          return Math.pow((in - deadzone)*(1.0/(1.0-deadzone)), 3);
      else 
          return -Math.pow((-in - deadzone)*(1.0/(1.0-deadzone)), 3);
  }
  /* 
  public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
  {
      return () -> interpolateNow(in.get(), deadzone);
  }

  public static double interpolateNow(double in, double deadzone)
  {
    if(Math.abs(in) < deadzone)
      return 0.0;
    else
      return in;
  }*/

    
  // Static reference to the robot class
  // Previously we used static subsystems, but this appears to break things in 2025
  // Use getRobot() to get the robot object
  private static RobotContainer robotReference;

  /**
   * Get a reference to the RobotContainer object in use
   * @return the active RobotContainer object
   */
  public static RobotContainer getRobot()
  {
    return robotReference;
  }

  /**
   * Set rumble for the specialist controller
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @return Instant command to set rumble strength
   */
  public Command vibrateSpecialist(RumbleType rumbleside, double rumblestrength )
  {
    return new InstantCommand(() -> specialist.setRumble(rumbleside, rumblestrength));
  }

  /**
   * Set rumble for the driver controller
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @return Instant command to set rumble strength
   */
  public Command vibrateDriver(RumbleType rumbleside, double rumblestrength )
  {
    return new InstantCommand(() -> driver.setRumble(rumbleside, rumblestrength));
  }

  /**
   * Rumble a controller while a command is running, stopping when the command finishes
   * @param controller Controller to rumble
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public Command vibrateWhile(CommandXboxController controller, RumbleType rumbleside, double rumblestrength, Command c)
  {
    return new ParallelRaceGroup(
      c,
      new FunctionalCommand(() -> controller.setRumble(rumbleside, rumblestrength),
      () -> controller.setRumble(rumbleside, rumblestrength),
      (interrupted) -> controller.setRumble(rumbleside, 0),
      () -> {return false;})
    );
  }

  /**
   * Rumble specials controller while a command is running, stopping when the command finishes
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public Command vibrateSpecialistWhile(RumbleType rumbleside, double rumblestrength, Command c)
  {
    return vibrateWhile(specialist, rumbleside, rumblestrength, c);
  }

  /**
   * Rumble driver controller while a command is running, stopping when the command finishes
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param c Command to run while rumbling
   * @return Instant command to set rumble strength
   */
  public Command vibrateDriverWhile(RumbleType rumbleside, double rumblestrength, Command c)
  {
    return vibrateWhile(driver, rumbleside, rumblestrength, c);
  }

  /**
   * Set specialist controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public Command vibrateSpecialistForTime(RumbleType rumbleside, double rumblestrength, double time)
  {
    return vibrateForTime(specialist, rumbleside, rumblestrength, time);
  }

  /**
   * Set driver controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public Command vibrateDriverForTime(RumbleType rumbleside, double rumblestrength, double time)
  {
    return vibrateForTime(driver, rumbleside, rumblestrength, time);
  }

  /**
   * Set a controller to rumble for a certain amount of time.
   * This isn't blocking- it schedules a separate command to end the rumbe later.
   * @param controller controller to rumble
   * @param rumbleside Which half of the controller to rumble, or both
   * @param rumblestrength Strength of the rumble, from 0.0 to 1.0
   * @param time How long to rumble for
   * @return Command to schedule the rumble
   */
  public Command vibrateForTime(CommandXboxController controller,RumbleType rumbleside, double rumblestrength, double time)
  {
    return new ScheduleCommand(vibrateWhile(controller, rumbleside, rumblestrength, new WaitCommand(time)));
  }
}
