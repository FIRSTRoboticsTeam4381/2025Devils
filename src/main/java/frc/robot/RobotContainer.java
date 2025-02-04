// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.PhotonCam;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
  
  // Controllers
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController specialist = new CommandXboxController(1);

  //Auto Chooser
  SendableChooser<Autos.PreviewAuto> autoChooser = new SendableChooser<>();

  // Subsystems
  public final Swerve swerve = new Swerve();

  
  //public final PhotonCam camA = new PhotonCam("Camera A", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/-4-Math.PI)) );
  //public final PhotonCam camB = new PhotonCam("Camera B", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
  //public final PhotonCam camC = new PhotonCam("Camera C", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
  //public final PhotonCam camD = new PhotonCam("Camera D", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );


  // Constructor: set up the robot! 
  public RobotContainer() {
    robotReference = this;

    // Set default commands here




    // Set up autonomous picker
    // Add any autos you want to be able to select below
    autoChooser.setDefaultOption("None", Autos.none());
    autoChooser.addOption("Test", Autos.testAuto());
    

    // Add auto controls to the dashboard
    SmartDashboard.putData("Choose Auto:", autoChooser);
    SmartDashboard.putData(CommandScheduler.getInstance());
    autoChooser.onChange((listener) -> listener.showPreview());
    SmartDashboard.putNumber("Start Delay",0);

    
    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
          //interpolateJoystick(driver::getLeftY,0.05),
          //interpolateJoystick(driver::getLeftX,0.05), 
          interpolateJoystick (driver::getRightX,0.05),
             true, driver.leftBumper()::getAsBoolean));
  }

  public Command getAutonomousCommand() {
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
  /*public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
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
  }*/

  public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
  {
      return () -> in.get();
  }

  public static double interpolateNow(double in, double deadzone)
  {
    if(Math.abs(in) < deadzone)
      return 0.0;
    else
      return in;
  }

    
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
