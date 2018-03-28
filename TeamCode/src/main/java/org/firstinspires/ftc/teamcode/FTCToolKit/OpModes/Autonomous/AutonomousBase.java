package org.firstinspires.ftc.teamcode.FTCToolKit.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional.HolonomicDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.Sensor;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Constants;
import static org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.HardwareComponents.*;
public class AutonomousBase extends LinearOpMode {
    public DriveTrain driveTrain;
    public AutonomousBase(Drivetrains driveTrains){
        if (driveTrains == Drivetrains.HOLONOMIC){
            driveTrain = new HolonomicDriveTrain(Constants.ANDYMARK_MOTOR_TICKS,Constants.inWheelDiameter, Sensor.REV);
        } else if (driveTrains == Drivetrains.MECANUM){
            driveTrain = new MecanumDriveTrain(Constants.ANDYMARK_MOTOR_TICKS,Constants.inWheelDiameter,Sensor.REV);
        } else if ()
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //This will never be run
    }
}
