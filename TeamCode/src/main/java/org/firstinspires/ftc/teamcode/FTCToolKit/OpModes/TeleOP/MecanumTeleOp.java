package org.firstinspires.ftc.teamcode.FTCToolKit.OpModes.TeleOP;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Constants;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Sensor;
import static org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.HardwareComponents.*;

@TeleOp(name = "Mecanum Bot Tele")
public class MecanumTeleOp extends TeleOpBase {
    public MecanumTeleOp(){
        super(Drivetrains.MECANUM, Constants.ANDYMARK_MOTOR_TICKS,Constants.inWheelDiameter, Sensor.REV);
    }
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        mecanumDriveTrain.driveControlled(gamepad1);
    }
}
