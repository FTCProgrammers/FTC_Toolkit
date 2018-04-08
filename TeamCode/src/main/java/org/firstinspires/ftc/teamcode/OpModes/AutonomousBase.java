package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.toolkit.Hardware.DriveTrain.TankDriveTrain;
public class AutonomousBase extends LinearOpMode {
    private TankDriveTrain driveTrain = new TankDriveTrain();
    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain.init(hardwareMap);
        waitForStart();
        driveTrain.rotate(1,45);
    }
}
