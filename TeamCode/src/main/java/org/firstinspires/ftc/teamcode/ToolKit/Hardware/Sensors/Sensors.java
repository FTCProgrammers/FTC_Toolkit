package org.firstinspires.ftc.teamcode.ToolKit.Hardware.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.ToolKit.Robot;

/**
 * Created by shaunaksarker on 2/4/18.
 */

public abstract class Sensors extends Robot {
    public ColorSensor sensorColor;
    public OpticalDistanceSensor ods;
    public DistanceSensor sensorDistance;

    public enum Color {
        BLUE,RED,ELSE
    }


    public abstract Color readColor();

}
