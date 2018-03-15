package org.firstinspires.ftc.teamcode.ToolKit.Hardware.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ToolKit.Robot;

/**
 * Created by shaunaksarker on 2/4/18.
 */

public abstract class Sensors extends Robot {
    public BNO055IMU imu;
    public ColorSensor sensorColor;
    public GyroSensor gyro;
    public OpticalDistanceSensor ods;
    public DistanceSensor sensorDistance;

    public enum Color {
        BLUE,RED,ELSE
    }


    public abstract Color readColor();

    public abstract double Heading();
}
