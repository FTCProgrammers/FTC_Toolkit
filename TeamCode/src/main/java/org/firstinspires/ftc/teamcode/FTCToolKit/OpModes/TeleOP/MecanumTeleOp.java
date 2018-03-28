package org.firstinspires.ftc.teamcode.FTCToolKit.OpModes.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional.MecanumDriveTrain;
@TeleOp(name = "Mecanum Bot Tele")
public class MecanumTeleOp extends OpMode {
    private MecanumDriveTrain driveTrain = new MecanumDriveTrain();
    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        driveTrain.setDefaultSpeed();
    }

    @Override
    public void loop() {
        driveTrain.driveControlled(gamepad1);
    }
}
