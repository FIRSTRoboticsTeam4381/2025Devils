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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AdvancedCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.PhotonCam;
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

  // Subsystems
  public final Swerve swerve = new Swerve();
  public final GroundIntake groundIntake = new GroundIntake();
  public final Intake armIntake = new Intake();
  public final SwingArm swingArm = new SwingArm();
  public final Extender extender = new Extender();
  public final Wrist wrist = new Wrist();
  public final Elevator elevator = new Elevator();
  public final Hang hang = new Hang();
  public final AdvancedCommands aCommands;
  

  
  //public final PhotonCam camA = new PhotonCam("Camera A", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/-4-Math.PI)) );
  //public final PhotonCam camB = new PhotonCam("Camera B", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
  //public final PhotonCam camC = new PhotonCam("Camera C", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
  //public final PhotonCam camD = new PhotonCam("Camera D", new Transform3d(new Translation3d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.3125),  Units.inchesToMeters(8.5)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );


  // Constructor: set up the robot! 
  public RobotContainer() 
  {
    robotReference = this;

    aCommands = new AdvancedCommands(robotReference);



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

  private void configureBindings() 
  {
    swerve.setDefaultCommand(new TeleopSwerve(swerve, 
            driver::getLeftY,
            driver::getLeftX,
          //interpolateJoystick(driver::getLeftY,0.05),
          //interpolateJoystick(driver::getLeftX,0.05), 
          interpolateJoystick (driver::getRightX,0.05),
             true, driver.leftBumper()::getAsBoolean));
      
      //specialist.a().onTrue(GroundIntake.intake());

      specialist.a().onTrue(aCommands.coralInOrOut());
      specialist.b().onTrue(aCommands.algaeInOrOut());

      // Elevator preset position controls
      specialist.povUp().onTrue(aCommands.l4()); // How do we determine the distance value here?
      specialist.povLeft().onTrue(aCommands.l3());
      specialist.povRight().onTrue(aCommands.l2());
      specialist.povDown().onTrue(aCommands.l1());

      //elevator joystick controls
      elevator.setDefaultCommand(elevator.joystickCtrl(specialist :: getLeftY));

      //wrist triggers  
      wrist.setDefaultCommand(wrist.joystickCtrl(specialist :: getLeftTriggerAxis, specialist :: getRightTriggerAxis));

      //swing joystick controls
      swingArm.setDefaultCommand(swingArm.swing(specialist :: getRightX));

      //extend joystick
      extender.setDefaultCommand(extender.extend(specialist :: getRightY));
      
    }

  public Command getAutonomousCommand() 
  {
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

}
