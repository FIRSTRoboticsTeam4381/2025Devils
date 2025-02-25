// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

@Logged
public final class Autos {
  

    public static Character prevChosenBranch;
    public static String station;
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

    public static PreviewAuto reefASideSelector(String autoName) {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(autoName),
                new ConditionalCommand(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()) , 
                    new SequentialCommandGroup(
                        //Start
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to A"))),
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to B"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to G"))),
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to H"))),
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to J"))),
                                Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to K"))),
                                Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStart to L")))
                            ), Autos::chosenPosition),
                        RobotContainer.getRobot().aCommands.l4L(), // Place Pos
                        RobotContainer.getRobot().intake.coralEjectL(), // Eject Coral
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenBranch + " to " + station)), // highly doubt this will work but it worth a try // change
                        RobotContainer.getRobot().aCommands.coralStationL(), // LEFT RIGHTS SUBJECT TO CHANGE
                        RobotContainer.getRobot().intake.coralIntakeL(),
                        new SelectCommand<Character>(
                            Map.ofEntries( // WILL BE A PATH THAT GOES FROM STATION TO REEF
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to A"))), // BLUE STATION 
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to B"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to G"))), // RED STAION ___________
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to H"))), // BLUE STATION _________
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to J"))),
                                Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to K"))),
                                Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to L"))) // BLUE STATION __________
                            ), Autos::chosenPosition
                        ),
                        RobotContainer.getRobot().aCommands.l4L(),
                        RobotContainer.getRobot().intake.coralEjectL()
                        //Finished
                    ), positionsTo::isEmpty).repeatedly()
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

    public static PreviewAuto reefPSideSelector(String autoName) {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(autoName),
                new ConditionalCommand(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()) , 
                    new SequentialCommandGroup(
                        //Start
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to A"))),
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to B"))),
                                Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to C"))),
                                Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to D"))),
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to G"))),
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStart to H")))
                            ), Autos::chosenPosition),
                        RobotContainer.getRobot().aCommands.l4L(), // Place Pos
                        RobotContainer.getRobot().intake.coralEjectL(), // Eject Coral // LEFT RIGHT SUBJECT TO CHANGE
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenBranch + " to pStation")), // highly doubt this will work but it worth a try // change
                        RobotContainer.getRobot().aCommands.coralStationL(),
                        RobotContainer.getRobot().intake.coralIntakeL(),
                        new SelectCommand<Character>(
                            Map.ofEntries( // WILL BE A PATH THAT GOES FROM STATION TO REEF 
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to A"))), // BLUE STATION 
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to B"))), // RED STATION ______
                                Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to C"))),
                                Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to D"))),
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to G"))), // RED STAION ___________
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to H"))) // BLUE STATION _________
                            ), Autos::chosenPosition
                        ),
                        RobotContainer.getRobot().aCommands.l4L(),
                        RobotContainer.getRobot().intake.coralEject()
                        //Finished
                    ), positionsTo::isEmpty).repeatedly()
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

    public static PreviewAuto reefMSelector(String autoName) {
        try {
            return new PreviewAuto(new SequentialCommandGroup(
                new PathPlannerAuto(autoName),
                new ConditionalCommand(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()) , 
                    new SequentialCommandGroup(
                        //Start
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("mStart to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("mStart to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("mStart to G"))),
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("mStart to H"))),
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("mStart to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("mStart to J")))
                            ), Autos::chosenPosition),
                        RobotContainer.getRobot().aCommands.l4L(), // Place Pos
                        RobotContainer.getRobot().intake.coralEjectL(), // Eject Coral
                        new DeferredCommand(() -> {try {
                            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenBranch + " to " + station));
                        } catch (FileVersionException | IOException | ParseException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                            return Commands.none();
                        }}, Set.of(RobotContainer.getRobot().swerve)), // highly doubt this will work but it worth a try // change
                        RobotContainer.getRobot().aCommands.coralStationR(),
                        RobotContainer.getRobot().intake.coralIntakeL(),
                        new SelectCommand<Character>(
                            Map.ofEntries( // WILL BE A PATH THAT GOES FROM STATION TO REEF
                                Map.entry('A', deferChooserPath(station, "to A")), // BLUE STATION  /// CONTINUE THIS NEXT
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to B"))), // RED STATION ______
                                Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to C"))),
                                Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to D"))),
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to G"))), // RED STAION ___________
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to H"))), // BLUE STATION _________
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to J"))),
                                Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to K"))),
                                Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " to L"))) // BLUE STATION __________
                            ), Autos::chosenPosition
                        ),
                        RobotContainer.getRobot().aCommands.l4L(),
                        RobotContainer.getRobot().intake.coralEjectL()
                        //Finished
                    ), positionsTo::isEmpty).repeatedly()
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
        return reefASideSelector("aStart");
    }

    public static PreviewAuto proSide() {
        return reefPSideSelector("pStart");
    }

    public static PreviewAuto middle() {
        return reefMSelector("middleStart");
    }

    public static Queue<Character> positionsTo = new LinkedList<>();

    public static Character chosenPosition() {
        prevChosenBranch = positionsTo.remove();
        if(prevChosenBranch == 'A' || prevChosenBranch == 'H'|| prevChosenBranch == 'I'
        || prevChosenBranch == 'J'|| prevChosenBranch == 'K'|| prevChosenBranch == 'L') {
            station = "aStation";
        } else {
            station = "pStation";
        }
        return prevChosenBranch;
    }

    public static void pickPosition() { // this may not be working?
        String chosenBranch = SmartDashboard.getString("Choose Reef Branch", "");
        positionsTo.clear();

        for(String n : chosenBranch.split(",")) {
            try {
                positionsTo.add(n.charAt(0));
            }catch(Exception e) {}
        }
    }


    public static DeferredCommand deferChooserPath(String station, String toWhere) {
        return new DeferredCommand(() -> {try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(station + " " + toWhere));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Commands.none();
        }}, Set.of(RobotContainer.getRobot().swerve));
    }


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
