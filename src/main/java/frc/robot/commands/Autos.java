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
import java.util.function.Supplier;

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
    public static String prevStation;
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


    public static PreviewAuto ProSideTrippple() {
        return new PreviewAuto("proSide E,B,A (trippple)");
    }
    public static PreviewAuto AntiSideTrippple() {
        return new PreviewAuto("antiSide J,A,B (trippple)");
    }


    public static PreviewAuto ProSideBasic() {
        return new PreviewAuto("proSide Basic (30s)");
    }
    public static PreviewAuto AntiSideBasic() {
        return new PreviewAuto("antiSide Basic (31s)");
    }


      //ProSide_D_C means D,C; Not ideal but it works. Same for _G_H = G,H below
    public static PreviewAuto ProSide_D_C() {
        return new PreviewAuto("proSide D,C");
    }

    public static PreviewAuto MiddleSide_G_H() {
        return new PreviewAuto("middleSide G,H");
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
                            new InstantCommand(() -> RobotContainer.getRobot().intake.ejectCoralL()),
                        new DeferredCommand(() -> {try {
                            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenBranch + " to " + station));
                        } catch (FileVersionException | IOException | ParseException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                            return Commands.none();
                        }}, Set.of(RobotContainer.getRobot().swerve)), // highly doubt this will work but it worth a try // change
                        new InstantCommand(() -> RobotContainer.getRobot().intake.intakeCoralL()),
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to A"))),
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to B"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to G"))),
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to H"))),
                                Map.entry('I', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to I"))),
                                Map.entry('J', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to J"))),
                                Map.entry('K', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to K"))),
                                Map.entry('L', AutoBuilder.followPath(PathPlannerPath.fromPathFile("aStation to L")))
                            ), Autos::chosenPosition
                        ),
                        new InstantCommand(() -> RobotContainer.getRobot().intake.ejectCoralL())
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
                            new InstantCommand(() -> RobotContainer.getRobot().intake.ejectCoralL()),
                        new DeferredCommand(() -> {try {
                            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenBranch + " to " + station));
                        } catch (FileVersionException | IOException | ParseException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                            return Commands.none();
                        }}, Set.of(RobotContainer.getRobot().swerve)), // highly doubt this will work but it worth a try // change
                        new InstantCommand(() -> RobotContainer.getRobot().intake.intakeCoralL()),
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('A', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to A"))),
                                Map.entry('B', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to B"))),
                                Map.entry('C', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to C"))),
                                Map.entry('D', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to D"))),
                                Map.entry('E', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to E"))),
                                Map.entry('F', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to F"))),
                                Map.entry('G', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to G"))),
                                Map.entry('H', AutoBuilder.followPath(PathPlannerPath.fromPathFile("pStation to H")))
                            ), Autos::chosenPosition
                        ),
                        new InstantCommand(() -> RobotContainer.getRobot().intake.ejectCoralL())
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
                        new InstantCommand(() -> RobotContainer.getRobot().intake.ejectCoralL()),
                        new DeferredCommand(() -> {try {
                            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(prevChosenBranch + " to " + station));
                        } catch (FileVersionException | IOException | ParseException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                            return Commands.none();
                        }}, Set.of(RobotContainer.getRobot().swerve)), // highly doubt this will work but it worth a try // change
                        new InstantCommand(() -> RobotContainer.getRobot().intake.intakeCoralL()),
                        new SelectCommand<Character>(
                            Map.ofEntries(
                                Map.entry('A', deferChooserPath(() -> station, "to A")), //Testing Defer Command
                                Map.entry('B', deferChooserPath(() -> station, "to B")),
                                Map.entry('C', deferChooserPath(() -> station, "to C")),
                                Map.entry('D', deferChooserPath(() -> station, "to D")),
                                Map.entry('E', deferChooserPath(() -> station, "to E")),
                                Map.entry('F', deferChooserPath(() -> station, "to F")),
                                Map.entry('G', deferChooserPath(() -> station, "to G")),
                                Map.entry('H', deferChooserPath(() -> station, "to H")),
                                Map.entry('I', deferChooserPath(() -> station, "to I")),
                                Map.entry('J', deferChooserPath(() -> station, "to J")),
                                Map.entry('K', deferChooserPath(() -> station, "to K")),
                                Map.entry('L', deferChooserPath(() -> station, "to L"))
                            ), Autos::chosenPosition
                        ),
                        new InstantCommand(() -> System.out.println(station)),
                        new InstantCommand(() -> RobotContainer.getRobot().intake.ejectCoralL())
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
        return reefASideSelector("AntiProcessor Start Chooser");
    }

    public static PreviewAuto proSide() {
        return reefPSideSelector("Processor Start Chooser");
    }

    public static PreviewAuto middle() {
        return reefMSelector("Middle Start Chooser");
    }

    public static Queue<Character> positionsTo = new LinkedList<>();

    public static Character chosenPosition() {
        prevChosenBranch = positionsTo.peek();
        if(prevChosenBranch == 'A' || prevChosenBranch == 'H'|| prevChosenBranch == 'I'
        || prevChosenBranch == 'J'|| prevChosenBranch == 'K'|| prevChosenBranch == 'L') {
            station = "aStation";
            System.out.println("ran IF statement: " + station);
        } else {
            station = "pStation";
            System.out.println("ran IF statement: " + station);
        }
        return positionsTo.remove();
    }

    public static void pickPosition() {
        String chosenBranch = SmartDashboard.getString("Choose Reef Branch", "");
        positionsTo.clear();

        for(String n : chosenBranch.split(",")) {
            try {
                positionsTo.add(n.charAt(0));
            }catch(Exception e) {}
        }
    }


    public static DeferredCommand deferChooserPath(Supplier<String> station, String toWhere) {
        return new DeferredCommand(() -> {try {
            System.out.println("Inside of the Defer Command in the map 2 the station is: " + station.get());
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(station.get() + " " + toWhere));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return Commands.none();
        }}, Set.of(RobotContainer.getRobot().swerve));
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
