package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.toolkit.Hardware.DriveTrain.OmniDirectional.MecanumDriveTrain;
public class TeleOP extends OpMode {
    MecanumDriveTrain driveTrain = new MecanumDriveTrain();
    @Override
    public void init() {
        driveTrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        driveTrain.driveControlled(gamepad1);
    }
}
