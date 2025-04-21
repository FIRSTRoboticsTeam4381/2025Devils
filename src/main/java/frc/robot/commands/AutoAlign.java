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
        
        // Blue snaps
        add(new Pose2d(3.261, 4.403, new Rotation2d(Radians.convertFrom(-90, Degrees))));//blue a
        add(new Pose2d(3.261, 3.917, new Rotation2d(Radians.convertFrom(-90, Degrees))));//blue b
        add(new Pose2d(3.584, 3.584, new Rotation2d(Radians.convertFrom(-30, Degrees))));//blue C
        add(new Pose2d(3.884, 2.892, new Rotation2d(Radians.convertFrom(-30, Degrees))));//blue d
        add(new Pose2d(4.833, 2.766, new Rotation2d(Radians.convertFrom(30, Degrees))));//blue e
        add(new Pose2d(5.059, 2.904, new Rotation2d(Radians.convertFrom(30, Degrees))));//blue f
        add(new Pose2d(5.850, 3.699, new Rotation2d(Radians.convertFrom(90, Degrees))));//blue g
        add(new Pose2d(5.802, 4.100, new Rotation2d(Radians.convertFrom(90, Degrees))));//blue h
        add(new Pose2d(4.951, 4.666, new Rotation2d(Radians.convertFrom(150, Degrees))));//blue i
        add(new Pose2d(5.083, 5.170, new Rotation2d(Radians.convertFrom(150, Degrees))));//blue j
        add(new Pose2d(4.100, 5.254, new Rotation2d(Radians.convertFrom(-150, Degrees))));//blue k
        add(new Pose2d(3.788, 5.122, new Rotation2d(Radians.convertFrom(-150, Degrees))));//blue l
        
        // Red snaps
        add(new Pose2d(12.17, 2.738, new Rotation2d(Radians.convertFrom(59.62, Degrees))));//red ?
        add(new Pose2d(11.49, 4.13, new Rotation2d(Radians.convertFrom(90, Degrees))));//red g
        add(new Pose2d(14.56, 3.88, new Rotation2d(Radians.convertFrom(180, Degrees))));//red a
        add(new Pose2d(12.43, 5.465, new Rotation2d(Radians.convertFrom(-60.17, Degrees))));//red e
        add(new Pose2d(13.94, 5.26, new Rotation2d(Radians.convertFrom(-119.12, Degrees))));//red c
        add(new Pose2d(14.55, 3.86, new Rotation2d(Radians.convertFrom(175.36, Degrees))));//red ?
        add(new Pose2d(14.06, 2.86, new Rotation2d(Radians.convertFrom(124.59, Degrees))));//red L
        add(new Pose2d(12.52, 2.61, new Rotation2d(Radians.convertFrom(63.16, Degrees))));//red d
        add(new Pose2d(11.5, 3.77, new Rotation2d(Radians.convertFrom(90, Degrees))));//red h
        add(new Pose2d(12.05, 5.29, new Rotation2d(Radians.convertFrom(-57.24, Degrees))));//red f
        add(new Pose2d(13.64, 5.41, new Rotation2d(Radians.convertFrom(-121.06, Degrees))));//red d
        add(new Pose2d(14.63, 4.16, new Rotation2d(Radians.convertFrom(176.9, Degrees))));//red b
        add(new Pose2d(13.75, 2.67, new Rotation2d(Radians.convertFrom(127.89, Degrees))));//red k
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
        // target = currentpose.nearest(snapPositions);

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