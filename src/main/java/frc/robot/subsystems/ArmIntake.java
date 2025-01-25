package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ArmIntake extends SubsystemBase{
    // Create motors
    public SparkMax intakeMotor1;
    public SparkMax intakeMotor2;
    public SparkMax intakeMotor3;

    public ArmIntake() {
        intakeMotor1 = new SparkMax(58, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(59, MotorType.kBrushless);
        intakeMotor3 = new SparkMax(60, MotorType.kBrushless);

        // ARM INTAKE contains 3 total motors

        SparkMaxConfig motor1Config = new SparkMaxConfig();

        motor1Config.smartCurrentLimit(30).idleMode(IdleMode.kCoast);

        intakeMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig motor2Config = new SparkMaxConfig();
        motor2Config.apply(motor1Config);
        motor2Config.follow(intakeMotor1, true); //Change (invert and maybe follow) based on oritentation and placement of the three motors
    
        SparkMaxConfig motor3Config = new SparkMaxConfig();
        motor3Config.apply(motor1Config);
        motor3Config.follow(intakeMotor1, true); // Here aswell

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

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        SmartDashboard.putData(this);
    }

}
