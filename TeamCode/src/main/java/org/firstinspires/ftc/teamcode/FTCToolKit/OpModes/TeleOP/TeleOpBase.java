package org.firstinspires.ftc.teamcode.FTCToolKit.OpModes.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional.HolonomicDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.TankDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.TwoWheeledDriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Constants;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.HardwareComponents;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Sensor;
import static org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.HardwareComponents.*;
public abstract class TeleOpBase extends OpMode{
    protected HolonomicDriveTrain holonomicDriveTrain;
    protected MecanumDriveTrain mecanumDriveTrain;
    protected TankDriveTrain tankDriveTrain;
    protected TwoWheeledDriveTrain twoWheeledDriveTrain;
    protected Drivetrains drivetrains;
    public TeleOpBase(HardwareComponents.Drivetrains drivetrains, int encoderTicks, double wheelDiameter, Sensor sensor){
        this.drivetrains = drivetrains;
        if (drivetrains == HardwareComponents.Drivetrains.HOLONOMIC){
            holonomicDriveTrain = new HolonomicDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.MECANUM){
            mecanumDriveTrain = new MecanumDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.TWOWHEELED){
            twoWheeledDriveTrain = new TwoWheeledDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.TANK){
            tankDriveTrain = new TankDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else {
            holonomicDriveTrain = null;
            mecanumDriveTrain = null;
            twoWheeledDriveTrain = null;
            tankDriveTrain = null;
        }

    }


    //Default Constructor is Tank
    public TeleOpBase(){
        tankDriveTrain = new TankDriveTrain(Constants.ANDYMARK_MOTOR_TICKS, Constants.inWheelDiameter, Sensor.MR);
    }

    /**
     * @param drivetrains
     * This is the specific drivetrain you are using in your auto
     */
    public TeleOpBase(HardwareComponents.Drivetrains drivetrains){
        int encoderTicks = Constants.ANDYMARK_MOTOR_TICKS;
        double wheelDiameter = Constants.inWheelDiameter;
        Sensor sensor = Sensor.MR;
        if (drivetrains == HardwareComponents.Drivetrains.HOLONOMIC){
            holonomicDriveTrain = new HolonomicDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.MECANUM){
            mecanumDriveTrain = new MecanumDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.TWOWHEELED){
            twoWheeledDriveTrain = new TwoWheeledDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.TANK){
            tankDriveTrain = new TankDriveTrain(encoderTicks,wheelDiameter,sensor);
        }
    }

    public TeleOpBase(HardwareComponents.Drivetrains drivetrains, Sensor sensor){
        int encoderTicks = Constants.ANDYMARK_MOTOR_TICKS;
        double wheelDiameter = Constants.inWheelDiameter;
        if (drivetrains == HardwareComponents.Drivetrains.HOLONOMIC){
            holonomicDriveTrain = new HolonomicDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.MECANUM){
            mecanumDriveTrain = new MecanumDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.TWOWHEELED){
            twoWheeledDriveTrain = new TwoWheeledDriveTrain(encoderTicks,wheelDiameter,sensor);
        } else if (drivetrains == HardwareComponents.Drivetrains.TANK){
            tankDriveTrain = new TankDriveTrain(encoderTicks,wheelDiameter,sensor);
        }
    }

    public void init(){
        if (drivetrains == HardwareComponents.Drivetrains.HOLONOMIC){
            holonomicDriveTrain.init(hardwareMap);
        } else if (drivetrains == HardwareComponents.Drivetrains.MECANUM){
            mecanumDriveTrain.init(hardwareMap);
        } else if (drivetrains == HardwareComponents.Drivetrains.TWOWHEELED){
            twoWheeledDriveTrain.init(hardwareMap);
        } else if (drivetrains == HardwareComponents.Drivetrains.TANK){
            tankDriveTrain.init(hardwareMap);
        }
    }

}
