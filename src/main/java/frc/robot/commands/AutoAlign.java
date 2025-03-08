// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import org.ejml.equation.Variable;
import org.photonvision.EstimatedRobotPose;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class AutoAlign extends Command 
{
    public final ArrayList<Pose2d> snapPositions= new ArrayList<Pose2d>(){{
        add(new Pose2d(0.726, 1.299, new Rotation2d(Radians.convertFrom(-60, Degrees))));
        add(new Pose2d(1.575, 0.666, new Rotation2d(Radians.convertFrom(-60, Degrees))));
        
    }};
    public Swerve swerve;
    private Pose2d target;
    public PIDController x;
    public PIDController y;
    public PIDController r;

    public AutoAlign(Swerve s){

        swerve = s;
        x = new PIDController(1.6, 0, 0);
        y = new PIDController(1.6, 0, 0);
        r = new PIDController(.04, 0, 0);
        r.enableContinuousInput(180,-180);
        addRequirements(swerve);
        
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        Pose2d currentpose = swerve.getPose();
        

        double bestDistance = Double.MAX_VALUE;

        for(Pose2d p : snapPositions){
            double distance = currentpose.getTranslation().getDistance(p.getTranslation());
            if (distance < bestDistance){
                target = p;
                bestDistance = distance;
            }
        }

        swerve.field.getObject("SnapToPose Target").setPose(target);
        
        x.setSetpoint(target.getX());
        y.setSetpoint(target.getY());
        r.setSetpoint(target.getRotation().getDegrees());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
       swerve.drive(new Translation2d(-getXPower(),-getYPower()), getRPower(), true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }

    public double getXPower(){
        return x.calculate(swerve.getPose().getX());
    }

    public double getYPower(){
        return y.calculate(swerve.getPose().getY());
    }

    public double getRPower(){
        return r.calculate(swerve.getPose().getRotation().getDegrees());
    }

}