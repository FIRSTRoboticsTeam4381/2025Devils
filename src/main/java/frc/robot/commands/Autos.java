// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public final class Autos {
  

    public static Character prevChosenAuto;
    public static String station;
    public static String startingPos;
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
    
    public static PreviewAuto ProSideBasic() {
        return new PreviewAuto("Proside Basic");
    }

    public static PreviewAuto reefPositionChooser(String autoName) {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(autoName),
                new ConditionalCommand(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()) , 
                    new SequentialCommandGroup(
                        //Start

                        /*new SelectCommand<String>(
                            Map.entry("Blue Corner Start Position", startPos = "Blue Corner"),
                            Map.entry("Red Corner Start Position", startPos = "Red Corner")),
                        new ConditionalCommand(null, null, )*/ // not work fo sho

                        // Need smth for start position **PRIORITY**
                        /*new SelectCommand<String>(
                            Map.ofEntries(
                                Map.entry("Left", startingPos = "Left"),
                                Map.entry("Middle", startingPos = "Middle"),
                                Map.entry("Right", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to A")))
                            ), Autos::chosenStartingPos
                        ),*/
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to A"))),
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to B"))),
                                Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to C"))),
                                Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to D"))),
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to G"))),
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to H"))),
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to J"))),
                                Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to K"))),
                                Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Start to L")))
                            ), Autos::chosenPosition),
                        RobotContainer.getRobot().aCommands.l4L(), // Place Pos
                        RobotContainer.getRobot().intake.coralEjectL(), // Eject Coral
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenAuto + " to " + station)), // highly doubt this will work but it worth a try // change
                        RobotContainer.getRobot().aCommands.coralStationR(),
                        RobotContainer.getRobot().intake.coralIntakeL(),
                        new SelectCommand<Character>(
                            Map.ofEntries( // WILL BE A PATH THAT GOES FROM STATION TO REEF
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to A"))), // BLUE STATION 
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to B"))), // RED STATION ______
                                Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to C"))),
                                Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to D"))),
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to G"))), // RED STAION ___________
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to H"))), // BLUE STATION _________
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to J"))),
                                Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to K"))),
                                Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("Station to L"))) // BLUE STATION __________
                            ), Autos::chosenPosition
                        ),
                        RobotContainer.getRobot().aCommands.l4L(),
                        RobotContainer.getRobot().intake.coralEjectL()
                        //Finished
                    ), positionTo::isEmpty).repeatedly()
                ), autoName
            );
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Autos.none();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Autos.none();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Autos.none();
        }
    }

    public static PreviewAuto antiSide() {
        return reefPositionChooser("aStart");
    }

    public static PreviewAuto proSide() {
        return reefPositionChooser("pStart");
    }

    public static PreviewAuto middle() {
        return reefPositionChooser("middleStart");
    }

    public static Queue<Character> positionTo = new LinkedList<>();

    public static Character chosenPosition() {
        prevChosenAuto = chosenPosition();
        if(prevChosenAuto == 'A' || prevChosenAuto == 'H'|| prevChosenAuto == 'I'
        || prevChosenAuto == 'J'|| prevChosenAuto == 'K'|| prevChosenAuto == 'L') {
            station = "aStation";
        } else {
            station = "pStation";
        }
        return positionTo.remove();
    }

    /*public static Queue<String> startPos = new LinkedList<>();

    public static String chosenStartingPos() {
        startingPos = chosenStartingPos();
        return startPos.remove();
    }*/  


    // IS SUPPOSED TO BE LEVEL CHOOSER Work In progress
    /*public static PreviewAuto reefLevelChooser() {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto("Level Chooser"),
                new ConditionalCommand(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()) , 
                    new SequentialCommandGroup(    
                    new SelectCommand<Integer>(
                            Map.ofEntries(
                                Map.entry(1, AutoBuilder.followPath()),
                                Map.entry(2, AutoBuilder.followPath()),
                                Map.entry(3, AutoBuilder.followPath()),
                                Map.entry(4, AutoBuilder.followPath())
                            ), Autos::chosenPosition)
                    ), positionTo::isEmpty).repeatedly()
                ), "Position Chooser");
        } catch (FileVersionException e) {
            e.printStackTrace();
            return Commands.none();
        } catch (IOException e) {
            e.printStackTrace();
            return Commands.none();
        } catch (ParseException e) {
            e.printStackTrace();
            return Commands.none();
        }
    } */

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
