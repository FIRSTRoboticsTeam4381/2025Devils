package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ArmIntake extends SubsystemBase{
    // Create motors
    public SparkMax intakeMotor1; // MOTORS 1 and 2 are algae, 2 follow one inverted
    public SparkMax intakeMotor2;
    public SparkMax intakeMotor3; // MOTOR 3 is for coral intake

    // public Double speed; // unused

    public ArmIntake() {
        intakeMotor1 = new SparkMax(58, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(59, MotorType.kBrushless);
        intakeMotor3 = new SparkMax(60, MotorType.kBrushless);

        // speed = 0.5; // Not used anywhere as of right now

        // ARM INTAKE contains 3 total motors

        SparkMaxConfig motor1Config = new SparkMaxConfig();

        motor1Config.smartCurrentLimit(30).idleMode(IdleMode.kCoast);

        intakeMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig motor2Config = new SparkMaxConfig();
        motor2Config.apply(motor1Config);
        motor2Config.follow(intakeMotor1, true); // might need to change invert 
    
        SparkMaxConfig motor3Config = new SparkMaxConfig();
        motor3Config.apply(motor1Config); // Intakes just algae thus it is serpate from motor 1 and 2

        intakeMotor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor3.configure(motor3Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        this.setDefaultCommand( // Stop motor I think
            new FunctionalCommand(() -> {},
                () -> intakeMotor1.set(0),
                (killed) -> {},
                () -> {return false;},
                this)
        );
    }

    // Intake and outake commands MAY REQUIRE change

    public Command algaeIntake() {
        return new InstantCommand(() -> intakeMotor1.set(1)); 
    }

    public Command algaeEject() {
        return new InstantCommand(() -> intakeMotor1.set(-1));
    }

    public Command coralIntake() {
        return new InstantCommand(() -> intakeMotor3.set(1)); 
    }

    public Command coralEject() {
        return new InstantCommand(() -> intakeMotor3.set(-1));
    }

    // STOP ---
    public Command algaeStop() {
        return new InstantCommand(() -> intakeMotor1.set(0));
    }

    public Command coralStop() {
        return new InstantCommand(() -> intakeMotor3.set(0));
    }
    // ------

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        SmartDashboard.putData(this);
    }

}
