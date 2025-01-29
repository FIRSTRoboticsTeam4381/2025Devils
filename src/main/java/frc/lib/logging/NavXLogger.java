// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** 
 * Custom logging class for NavX devices, including alerts for important conditions.
 */
@CustomLoggerFor(AHRS.class)
public class NavXLogger extends ClassSpecificLogger<AHRS>
{

    private Alert disconnected = new Alert("NavX disconnected!", AlertType.kError);
    private Alert isCalibrating = new Alert("NavX calibrating, please wait...", AlertType.kError);
    private Alert isMagCalibrating = new Alert("NavX magnometer calibrating, please wait...", AlertType.kError);
    private Alert magError = new Alert("NavX EM interference problem!", AlertType.kWarning);


    public NavXLogger()
    {
        super(AHRS.class);
    }

    @Override
    protected void update(EpilogueBackend backend, AHRS navx) {
        
        // Set alert status
        disconnected.set(!navx.isConnected());
        backend.log("Connected", navx.isConnected());

        isCalibrating.set(navx.isCalibrating());
        backend.log("Calibrating", navx.isCalibrating());

        isMagCalibrating.set(navx.isMagnetometerCalibrated());
        backend.log("Magnometer Calibrated", navx.isMagnetometerCalibrated());

        magError.set(navx.isMagneticDisturbance());
        backend.log("Magnetic Disturbance", navx.isMagneticDisturbance());

        
        // Log other data
        backend.log("Temperature", navx.getTempC());
        
        backend.log("Yaw", navx.getYaw());
        backend.log("Pitch", navx.getPitch());
        backend.log("Roll", navx.getRoll());

        backend.log("Compass", navx.getCompassHeading());

        backend.log("Fused Heading", navx.getFusedHeading());

        backend.log("Acceleration/X", navx.getWorldLinearAccelX());
        backend.log("Acceleration/Y", navx.getWorldLinearAccelY());
        backend.log("Acceleration/Z", navx.getWorldLinearAccelZ());

        backend.log("RotationRate/X",navx.getRawGyroX());
        backend.log("RotationRate/Y",navx.getRawGyroY());
        backend.log("RotationRate/Z",navx.getRawGyroZ());

        backend.log("Magnometer/X", navx.getRawMagX());
        backend.log("Magnometer/Y", navx.getRawMagY());
        backend.log("Magnometer/Z", navx.getRawMagZ());

        backend.log("Velocity/X", navx.getRobotCentricVelocityX());
        backend.log("Velocity/Y", navx.getRobotCentricVelocityY());
        backend.log("Velocity/Z", navx.getRobotCentricVelocityZ());

    }

}
