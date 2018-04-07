package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCToolKit.Robot;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Sensor;

/**
 * Created by shaunaksarker on 3/28/18.
 */

public class RobotColorSensor extends Robot{
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public OpticalDistanceSensor ods;
    protected Sensor sensor;
    protected Sensor sensorOds;
    public enum Color {
        BLUE,RED,ELSE
    }

    public RobotColorSensor(Sensor sensor){
        this.sensor = sensor;
    }
    //Default Constructor lead to MR
    public RobotColorSensor(){
        sensor = Sensor.MR;
    }
    @Override
    public void init(HardwareMap hwMap) {
        if (sensor == Sensor.MR){
            sensorColor = hwMap.colorSensor.get("cs");
            ods = hwMap.opticalDistanceSensor.get("ods");
        } else if (sensor == Sensor.REV){
            sensorColor = hwMap.get(ColorSensor.class, "cds");
            sensorDistance = hwMap.get(DistanceSensor.class, "cds");
        }
    }

    @Override
    protected void logTelemetry(Telemetry telemetry) {

    }

}
