package frc.robot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Custom logging class for Spark Max/Flex devices to log faults
 * and warnings into a nice readable string.
 */
@CustomLoggerFor(SparkBase.class)
public class SparkFaultLogger extends ClassSpecificLogger<SparkBase> {

    private Alert motorFaults;
    private Alert motorWarnings;

  public SparkFaultLogger() {
    super(SparkBase.class);
    motorFaults = new Alert("null", AlertType.kError);
    motorWarnings = new Alert("null", AlertType.kWarning);
  }

  @Override
  public void update(EpilogueBackend backend, SparkBase motor) {

    Faults f = motor.getFaults();
    String s = "";

    if(f.can)
        s += "CAN, ";
    if(f.escEeprom)
        s += "escEeprom, ";
    if(f.firmware)
        s += "firmware, ";
    if(f.gateDriver)
        s += "gateDriver, ";
    if(f.motorType)
        s += "motorType, ";
    if(f.sensor)
        s += "sensor, ";
    if(f.temperature)
        s += "temperature, ";
    if(f.other)
        s += "other, ";

    backend.log("Faults", s);

    if(s.length() > 0)
    {
        motorFaults.setText("Spark "+motor.getDeviceId()+" faults: "+s);
        motorFaults.set(true);
    }
    else
        motorFaults.set(false);


    Warnings w = motor.getWarnings();
    s = "";
    if(w.brownout)
        s += "brownout, ";
    if(w.escEeprom)
        s += "escEeprom, ";
    if(w.extEeprom)
        s += "extEeprom, ";
    if(w.hasReset)
        s += "hasReset, ";
    if(w.other)
        s += "other, ";
    if(w.overcurrent)
        s += "overcurrent, ";
    if(w.sensor)
        s += "sensor, ";
    if(w.stall)
        s += "stall, ";

    backend.log("Warnings", s);

    if(s.length() > 0)
    {
        motorWarnings.setText("Spark "+motor.getDeviceId()+" warnings: "+s);
        motorWarnings.set(true);
    }
    else
        motorWarnings.set(false);
  }
}
