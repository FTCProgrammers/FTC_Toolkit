package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCToolKit.Robot;

public class MRSensorClass extends Robot {
    public ColorSensor sensorColor;
    public OpticalDistanceSensor ods;
    public enum Color {
        BLUE,RED,ELSE
    }
    @Override
    public void init(HardwareMap hwMap) {
        sensorColor = hwMap.colorSensor.get("cs");
        ods = hwMap.opticalDistanceSensor.get("ods");
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        telemetry.addData("Color", readColor());
    }


    public Color readColor() {
        boolean blue = false, red = false;
        if(sensorColor.red() > sensorColor.blue()){
            blue = false;
            red = true;
        } else if (sensorColor.blue() > sensorColor.red()){
            blue = true;
            red = false;
        }
        return blue ? Color.BLUE : (red ? Color.RED : Color.ELSE);
    }
}