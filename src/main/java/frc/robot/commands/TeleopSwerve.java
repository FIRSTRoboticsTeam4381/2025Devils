package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command{
    
    private double rotation;
    private Translation2d translation;
    private boolean openLoop;

    private Swerve s_Swerve;
    //private CommandPS4Controller controller;
    //private CommandPS4Controller controller2;
    private Supplier<Double> forward;
    private Supplier<Double> leftright;
    private Supplier<Double> rotate;
    private Supplier<Boolean> slow;

    //static StructArrayPublisher<Translation2d> pointPub = NetworkTableInstance.getDefault()
    //  .getStructArrayTopic("joystick", Translation2d.struct).publish();
    
    
    /**
     * Command for driver control of a swerve drive. 
     * Can also be used in other ways by passing in different suppliers.
     * @param s_Swerve Swerve drive object to control
     * @param forward Supplier for forward/back motion (likely a joystick)
     * @param leftright Supplier for left/right motion (likely a joystick)
     * @param rotate Supplier for rotation control (likely a joystick)
     * @param openLoop Whether to use open or closed loop math
     * @param slow Supplier for whether to activate slow mode
     */
    public TeleopSwerve(Swerve s_Swerve, Supplier<Double> forward, Supplier<Double> leftright, Supplier<Double> rotate, boolean openLoop, Supplier<Boolean> slow){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.forward = forward;
        this.leftright = leftright;
        this.rotate = rotate;
        this.openLoop = openLoop;
        this.slow = slow;

    }

    @Override
    public void execute(){
        double yAxis = -forward.get();
        double xAxis = -leftright.get();
        double rAxis = -rotate.get();

        /* Slow Trigger */
        double slowdown = (slow.get() ? .25 : 1);
        yAxis *= slowdown;
        xAxis *= slowdown;
        rAxis *= slowdown;

        /* Calculates inputs for swerve subsystem */
        //translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = RobotContainer.interpolateNow(rAxis, Constants.stickDeadband) * Constants.Swerve.maxAngularVelocity;
        //s_Swerve.drive(translation, rotation, true, openLoop);


        Translation2d x = new Translation2d(yAxis, xAxis);

        Translation2d y = new Translation2d(RobotContainer.interpolateNow(x.getNorm(), Constants.stickDeadband), x.getAngle());

        // Debugging info for analyzing joystick behavior with different interpolation functions
        // Uncomment and use AdvantageScope points view to display
        /*x = x.times(RobotContainer.interpolateNow(x.getNorm(), 0.1));
        pointPub.set(new Translation2d[] {
            new Translation2d(yAxis, xAxis),
            new Translation2d(0, y.getNorm()),
            new Translation2d(RobotContainer.interpolateNow(xAxis, 0.1),0),
            y
        });*/

        translation = y.times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, rotation, true, openLoop);


    }
}
