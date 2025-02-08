// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public final class Autos {
  
  // TODO register commands in subsystem constructores using NamedCommands.registerCommand()

    // Test autonomous mode
    public static PreviewAuto testAuto(){
        return new PreviewAuto("Test");
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static PreviewAuto none(){
        return new PreviewAuto(Commands.none());
    }

    public static PreviewAuto OutTheWayBlue() {
        return new PreviewAuto("Out The Way Blue");
    }

    public static PreviewAuto OutTheWayRed() {
        return new PreviewAuto("Out The Way Red");
    }

    public static PreviewAuto RedD() {
        return new PreviewAuto("Red Coral D");
    }
    
    public static PreviewAuto BlueA() {
        return new PreviewAuto("Blue Coral A");
    }

    public static PreviewAuto BlueSimpleI() {
        return new PreviewAuto("Blue Simple I");
    }

    public static PreviewAuto RedSimpleF() {
        return new PreviewAuto("Red Simple F");
    }

    // TODO add pathplanner autos here. Example:
    //public static PreviewAuto Front3Note(){
    //    return new PreviewAuto("Front3NoteAuto");
    //}

    /* If you want to make a more complex auto using commands,
    *  PreviewAuto can also accept (Command, String), which will
    *  run Command while still showing a path preview for the path
    *  with filename String.
    */ 

    public static class PreviewAuto {
        public Command auto;
        public ArrayList <Pose2d> preview = new ArrayList<>();


        public void showPreview() {
            if (preview != null) {
                RobotContainer.getRobot().swerve.field.getObject("path").setPoses(preview);
            }
        }

        public PreviewAuto(Command a) {
            auto = a;
        }

        public PreviewAuto(String s) {
            auto = new PathPlannerAuto(s);

            try
            {
                for(PathPlannerPath p : PathPlannerAuto.getPathGroupFromAutoFile(s))
                {
                    preview.addAll(p.getPathPoses());
                }
            }
            catch(Exception e)
            {
                e.printStackTrace();
                DriverStation.reportError("Failed to load autonomous from PathPlanner file!", false);
            }
        }

        public PreviewAuto(Command c, String s) {
            auto = c;

            try
            {
                for(PathPlannerPath p : PathPlannerAuto.getPathGroupFromAutoFile(s))
                {
                    preview.addAll(p.getPathPoses());
                }
            }
            catch(Exception e)
            {
                e.printStackTrace();
                DriverStation.reportError("Failed to load autonomous from PathPlanner file!", false);
            }
        }

    }

}
