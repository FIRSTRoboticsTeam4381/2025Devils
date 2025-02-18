// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    DataLogManager.logConsoleOutput(true);
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
          Map.entry(41, "Swerve/BL/Angle"),
          Map.entry(50, "Elevator/motor1"),
          Map.entry(51, "Elevator/motor2"),
          Map.entry(52, "SwingArm/motor1"),
          Map.entry(53, "SwingArm/motor2"),
          Map.entry(54, "Extender/motor1"),
          Map.entry(56, "Wrist/motor1"),
          Map.entry(58, "Intake/intake1"),
          Map.entry(59, "Intake/intake2"),
          Map.entry(60, "Intake/intake3"),
          Map.entry(61, "Hang/motor1")

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

    logRioData();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Check if joysticks aren't zeroed
    // This should be ambiguous about what type of controller is plugged in
    driverControllerUnzeroed.set(!isJoystickZeroed(0));
    specialsControllerUnzeroed.set(!isJoystickZeroed(1));
  }

  /*
   * Helper to check 
   */
  private boolean isJoystickZeroed(int controller)
  {
    // How many axes are on this controller?
    int axes = DriverStation.getStickAxisCount(controller);
    for(int j=0; j<axes; j++)
    {
      if(Math.abs(DriverStation.getStickAxis(controller, j)) > 0.15)
      {
        // If joystic isn't being touched, it was probably plugged in with a stick held
        return false;
      }
    }

    // Didn't find an axis out of position
    return true;
  }

  @Override
  public void disabledExit() {
    driverControllerUnzeroed.set(false);
    specialsControllerUnzeroed.set(false);
  }

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

  // Alerts for RoboRIO problems
  private Alert brownout = new Alert("RIO browning out!", AlertType.kError);
  private Alert commsDisabled = new Alert("RIO reporting communications problems with DS!", AlertType.kWarning);

  private Alert fault3v = new Alert("", AlertType.kWarning);
  private Alert out3v = new Alert("RIO 3.3v rail dead!", AlertType.kError);
  private Alert fault5v = new Alert("", AlertType.kWarning);
  private Alert out5v = new Alert("RIO 5v rail dead!", AlertType.kError);
  private Alert fault6v = new Alert("", AlertType.kWarning);
  private Alert out6v = new Alert("RIO 6v rail dead!", AlertType.kError);

  private Alert canOut = new Alert("", AlertType.kError);
  private Alert canTx = new Alert("", AlertType.kError);
  private Alert canRx = new Alert("", AlertType.kError);
  private Alert canTxFull = new Alert("", AlertType.kError);
  
  private Alert driverControllerUnplugged = new Alert("Driver controller unplugged!", AlertType.kError);
  private Alert specialsControllerUnplugged = new Alert("Specials controller unplugged!", AlertType.kError);

  private Alert driverControllerUnzeroed = new Alert("Driver controller joysticks may not be zeroed! Try reconnecting it.", AlertType.kWarning);
  private Alert specialsControllerUnzeroed = new Alert("Specials controller joysticks may not be zeroed! Try reconnecting it.", AlertType.kWarning);

  /*
   * Log data about the health of the RoboRIO and display alerts if there are problems.
   */
  public void logRioData() {
    // Brownout
    boolean b = RobotController.isBrownedOut();
    brownout.set(b);
    SmartDashboard.putBoolean("RIO/brownout", b);

    // Communication losses
    int i = RobotController.getCommsDisableCount();
    SmartDashboard.putNumber("RIO/comms disabled count", i);
    if(i > 0)
      commsDisabled.set(true);

    // Battery voltage
    SmartDashboard.putNumber("RIO/Battery voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("RIO/Current", RobotController.getInputCurrent());
    SmartDashboard.putNumber("RIO/Voltage", RobotController.getInputVoltage());

    // Aux power rails
    SmartDashboard.putNumber("RIO/3.3v/voltage", RobotController.getVoltage3V3());
    SmartDashboard.putNumber("RIO/3.3v/current", RobotController.getCurrent3V3());
    SmartDashboard.putNumber("RIO/5v/voltage", RobotController.getVoltage5V());
    SmartDashboard.putNumber("RIO/5v/current", RobotController.getCurrent5V());
    SmartDashboard.putNumber("RIO/6v/voltage", RobotController.getVoltage6V());
    SmartDashboard.putNumber("RIO/6v/current", RobotController.getCurrent6V());

    // 3.3v rail
    i = RobotController.getFaultCount3V3();
    SmartDashboard.putNumber("RIO/3.3v/shorts", i);
    if(i > 0)
    {
      fault3v.setText("RIO 3.3v rail short detected. Count: "+i);
      fault3v.set(true);
    }
    b = RobotController.getEnabled3V3();
    SmartDashboard.putBoolean("RIO/3.3v/on", b);
    out3v.set(!b);

    // 5v rail
    i = RobotController.getFaultCount5V();
    SmartDashboard.putNumber("RIO/5v/shorts", i);
    if(i > 0)
    {
      fault5v.setText("RIO 5v rail short detected. Count: "+i);
      fault5v.set(true);
    }
    b = RobotController.getEnabled5V();
    SmartDashboard.putBoolean("RIO/5v/on", b);
    out5v.set(!b);

    // 6v rail
    i = RobotController.getFaultCount6V();
    SmartDashboard.putNumber("RIO/6v/shorts", i);
    if(i > 0)
    {
      fault6v.setText("RIO 6v rail short detected. Count: "+i);
      fault6v.set(true);
    }
    b = RobotController.getEnabled6V();
    SmartDashboard.putBoolean("RIO/6v/on", b);
    out6v.set(!b);

    // Temp
    SmartDashboard.putNumber("RIO/Temperature", RobotController.getCPUTemp());

    // CAN network status
    CANStatus c = RobotController.getCANStatus();

    i = c.busOffCount;
    SmartDashboard.putNumber("RIO/CAN/busOff", i);
    if(i > 0)
    {
      canOut.setText("CAN network is/was offline! Count: "+i);
      canOut.set(true);
    }

    i = c.receiveErrorCount;
    SmartDashboard.putNumber("RIO/CAN/Receive Error", i);
    if(i > 0)
    {
      canRx.setText("CAN network failing to receive packets! Count: "+i);
      canRx.set(true);
    }

    i = c.transmitErrorCount;
    SmartDashboard.putNumber("RIO/CAN/Transmit Error", i);
    if(i > 0)
    {
      canTx.setText("CAN network failing to transmit packets! Count: "+i);
      canTx.set(true);
    }

    i = c.txFullCount;
    SmartDashboard.putNumber("RIO/CAN/txFull", i);
    if(i > 0)
    {
      canTxFull.setText("CAN network transmit buffer full! Count: "+i);
      canTxFull.set(true);
    }

    SmartDashboard.putNumber("RIO/CAN/percent utilization", c.percentBusUtilization);
    


    // Is driver station connected?
    SmartDashboard.putBoolean("RIO/DS Connected", DriverStation.isDSAttached());


    // Are controllers connected?
    driverControllerUnplugged.set(!DriverStation.isJoystickConnected(0));
    specialsControllerUnplugged.set(!DriverStation.isJoystickConnected(1));
  
  }
}
