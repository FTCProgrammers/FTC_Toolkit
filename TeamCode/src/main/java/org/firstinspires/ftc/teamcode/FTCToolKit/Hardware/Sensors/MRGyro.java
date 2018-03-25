package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.TwoWheeledDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Robot;

/**
 * Created by shaunaksarker on 3/24/18.
 */

public class MRGyro extends Robot {
    private GyroSensor gyro;
    private TwoWheeledDriveTrain driveTrain;
    @Override
    public void init(HardwareMap hwMap) {
        gyro = hwMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    public double getHeading(){
        return gyro.getHeading();
    }
    /**
     * @Param angle, this is the heading that you what the gyro to turn to
     * NOTE: This does not mean it will turn that angle, however it will turn until it reaches that angle as its heading
     */

    public void rotate(double power,double angle) throws InterruptedException{
        while (getHeading() != angle){
            driveTrain.drive(power,-power);
        }
    }

    public void turn(double power, double angle){
        double heading = getHeading() + angle;
        while (getHeading() != heading){
            driveTrain.drive(power,-power);
        }
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {

    }
}
