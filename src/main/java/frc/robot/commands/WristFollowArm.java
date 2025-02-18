// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwingArm;
import frc.robot.subsystems.Wrist;


public class WristFollowArm extends Command {
  private Wrist wrist;
  private SwingArm arm;
  private double holdingAngle;

  /** Creates a new WristFollowArm. */
  public WristFollowArm(Wrist wrist, SwingArm arm, double holdingAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist=wrist;
    this.arm=arm;
    this.holdingAngle=holdingAngle;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO negatives may need to be flip flopped because I'm not sure which direction is which for these encoders

    wrist.setPositionReference(holdingAngle-arm.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
