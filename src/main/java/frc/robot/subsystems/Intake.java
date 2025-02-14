package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

@Logged
public class Intake extends SubsystemBase
{
    public SparkMax intake1; // MOTORS 1 and 2 are algae, 2 follow one inverted
    //public SparkMax intake2;
    public SparkMax intake3; // MOTOR 3 is for coral intake
    public double topSpeed = 5000;
    public double v = 0;
    public boolean hasAlgae = false;
    public final double ALGAE_SPIKE = 1500;

    private double[] currentTracker = new double[25];
    private double averageCurrent;

  
    public SparkLimitSwitch coralSensor;

    //public Supplier<Boolean> hasAlgae;
    //public Supplier<Boolean> hasCoral;

    // public Double speed; (unused)

  // create ArmIntake
  public Intake() 
  {
    intake1 = new SparkMax(58, MotorType.kBrushless);
    //intake2 = new SparkMax(59, MotorType.kBrushless);
    intake3 = new SparkMax(60, MotorType.kBrushless);
    
     // CHANGE CHANNELS LATER
    coralSensor = intake3.getForwardLimitSwitch();

    // speed = 0.5; // Not used anywhere as of right now

    // ARM INTAKE contains 3 total motors
    SparkMaxConfig intake1Config = new SparkMaxConfig();
      intake1Config.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
        intake1.configure(intake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intake2Config = new SparkMaxConfig();
      intake2Config.apply(intake1Config);
      intake2Config.follow(intake1, true); // might need to change invert 
        //intake2.configure(intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig motor3Config = new SparkMaxConfig();
      motor3Config.apply(intake1Config)
      .limitSwitch.forwardLimitSwitchEnabled(false); // Intakes just algae thus it is serpate from motor 1 and 20
        intake3.configure(motor3Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    

    this.setDefaultCommand( // Stop motor I think
        new FunctionalCommand(() -> {},
            () -> {intake1.set(0); intake3.set(0);},
            (killed) -> {},
            () -> {return false;},
            this)
    );

    intake1.getEncoder().getVelocity();
    intake3.getEncoder().getVelocity();
  }


// Intake and outake commands MAY REQUIRE change:
  // algae
  public Command algaeIntake() 
  {
    return new InstantCommand(() -> intake3.set(0.5), this); 
  }
  public Command algaeEject() 
  {
    return new InstantCommand(() -> intake3.set(-0.5), this);
  }


  // coral
  public Command coralIntake() 
  {
    return new InstantCommand(() -> intake1.set(-0.4), this); 
  }
  public Command coralEject() 
  {
    return new InstantCommand(() -> intake1.set(0.4), this);
  }


  // stop
  public Command algaeStop() 
  {
    return new InstantCommand(() -> intake3.set(0), this);
  }
  public Command coralStop() 
  {
    return new InstantCommand(() -> intake1.set(0), this);
  }


  public Command intakeAlgae() {
    return new RepeatCommand(
      algaeIntake()
    ).until(
      //() -> intake3.get() < (topSpeed - ALGAE_SPIKE)
      () -> averageCurrent > 30
    ).andThen(
      algaeStop()
    ).andThen(
      () -> hasAlgae = true
    );
  }

  public Command intakeCoral() {
    return new RepeatCommand(
      coralIntake()
    ).until(
      () -> coralSensor.isPressed()
    ).andThen(
      coralStop()
    ).andThen(
      RobotContainer.getRobot().vibrateSpecialistForTime(RumbleType.kRightRumble, 0.6, 1)
    );
  }

  public Command ejectAlgae() {
    
    return new RepeatCommand(
      algaeEject()
    ).until(
      () -> intake3.getEncoder().getVelocity() > topSpeed
    ).andThen(
      algaeStop()
    ).andThen(
      () -> hasAlgae = false
    );
  }

  public Command ejectCoral() {
    return new RepeatCommand(
      coralEject()
    ).until(
      () -> !coralSensor.isPressed() 
    ).andThen(
      coralStop()
    ).andThen(
      RobotContainer.getRobot().vibrateSpecialistForTime(RumbleType.kLeftRumble, 0.6, 1)
    );
  }

  public Command coralInOrOut() {
    return new ConditionalCommand(ejectCoral(), intakeCoral(), coralSensor::isPressed);
  }

  public Command algaeInOrOut() {
    return new ConditionalCommand(ejectAlgae(), intakeAlgae(), () -> hasAlgae);
  }
  
  @Override
  public void periodic() 
  {
    //This method will be called once per scheduler run
      SmartDashboard.putData(this);
      SmartDashboard.putNumber("Average Current", averageCurrent);
      SmartDashboard.putBoolean("Has Algae", hasAlgae);
      /* 
      v = intake3.get();
      if(v > topSpeed) 
      topSpeed = v;
      */

      for(int i = currentTracker.length-1; i > 0; i--){
        currentTracker[i] = currentTracker[i-1];
      }
      currentTracker[0]=intake3.getOutputCurrent();
      double runningSum=0;
      for(int i = 0; i < currentTracker.length; i++){
        runningSum+=currentTracker[i];
      }
      averageCurrent = runningSum/25;
  }
}
