package org.firstinspires.ftc.teamcode.ToolKit.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by shaunaksarker on 2/4/18.
 */

public class MRSensorClass extends Sensors {
    @Override
    public void init(HardwareMap hwMap) {
        gyro = hwMap.gyroSensor.get("gyro");
        sensorColor = hwMap.colorSensor.get("cs");
        ods = hwMap.opticalDistanceSensor.get("ods");
        gyro.calibrate();
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading", Heading());
        telemetry.addData("Color", readColor());
    }


    @Override
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

    @Override
    public double Heading() {
        return gyro.getHeading();
    }
}
