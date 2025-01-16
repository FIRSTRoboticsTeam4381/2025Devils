// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    // Forward ports for pis
    PortForwarder.add(5801, "pi1.local", 5800);
    PortForwarder.add(5802, "pi2.local", 5800);

    for (int port = 1180; port <= 1190; port++){
      PortForwarder.add(port, "pi1.local", port);
    }

    // Start logging system
    DataLogManager.start();

    // Start logging REV can network traffic
    URCL.start(Map.ofEntries(
          Map.entry(10, "Swerve/FL/Drive"),
          Map.entry(11, "Swerve/FL/Angle"),
          Map.entry(20, "Swerve/FR/Drive"),
          Map.entry(21, "Swerve/FR/Angle"),
          Map.entry(30, "Swerve/BR/Drive"),
          Map.entry(31, "Swerve/BR/Angle"),
          Map.entry(40, "Swerve/BL/Drive"),
          Map.entry(41, "Swerve/BL/Angle")
      ));

    // Start logging driver station data
    DriverStation.startDataLog(DataLogManager.getLog());
    
    // Start epilogue logging
    Epilogue.bind(this);

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
