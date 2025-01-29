// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SparkPosition;

@Logged
public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  public SparkFlex wristMotor1; // Currently assuming one motor on the wrist

  public AbsoluteEncoder position;

  public Supplier<Double> joystickValue;

  public Wrist() {
    wristMotor1 = new SparkFlex(56, MotorType.kBrushless);

    SparkFlexConfig motor1Config = new SparkFlexConfig();

    motor1Config.smartCurrentLimit(30).idleMode(IdleMode.kCoast);

    wristMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.setDefaultCommand( // Stop motor I think
        new FunctionalCommand(() -> {
        },
            () -> wristMotor1.set(0),
            (killed) -> {
            },
            () -> {
              return false;
            },
            this));

  }

  public Command joystickControl(Supplier<Double> joystickValue) {
    return new RepeatCommand(
        new InstantCommand(() -> wristMotor1.set(joystickValue.get()), this));
  }

  public Command wristToAngle(double position) {
    return new SparkPosition(wristMotor1, position, 1.0, this).withName("Wrist to" + position); // Will add positions later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
