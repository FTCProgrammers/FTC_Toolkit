package org.firstinspires.ftc.teamcode.FTCToolKit.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.TankDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Sensor;

public abstract class AutonomousBase extends LinearOpMode {
    protected TankDriveTrain driveTrain;

    public AutonomousBase(int encoderTicks, double wheelDiameter, Sensor sensor){
        driveTrain = new TankDriveTrain(encoderTicks,wheelDiameter,sensor);
    }

    public AutonomousBase(int encoderTicks, double wheelDiameter){
        driveTrain = new TankDriveTrain(encoderTicks,wheelDiameter);
    }

}
