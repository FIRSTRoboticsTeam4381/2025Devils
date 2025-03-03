// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class HangCam extends SubsystemBase {

  private PhotonCamera cam;

  /** Creates a new DriverCam. */
  public HangCam(String name) {

    cam = new PhotonCamera(name);

    // Set camera to driver mode
    cam.setDriverMode(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}