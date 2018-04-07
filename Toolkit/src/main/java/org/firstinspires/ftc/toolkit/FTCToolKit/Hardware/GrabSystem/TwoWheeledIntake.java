package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.GrabSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCToolKit.Robot;

public class TwoWheeledIntake extends Robot {
    private double maxSpeed;
    private boolean encoder;
    private DcMotor rightIntake;
    private DcMotor leftIntake;

    public TwoWheeledIntake(double maxSpeed){
        this.maxSpeed = maxSpeed;
    }

    public TwoWheeledIntake(){
        maxSpeed = 1.0;
    }
    @Override
    public void init(HardwareMap hwMap) {
        rightIntake = hwMap.dcMotor.get("ri");
        leftIntake = hwMap.dcMotor.get("li");
    }

    private void setMode(DcMotor.RunMode runMode){
        rightIntake.setMode(runMode);
        leftIntake.setMode(runMode);
    }

    private void hungryRobits(double power){
            if (power > maxSpeed){
                rightIntake.setPower(maxSpeed);
                leftIntake.setPower(maxSpeed);
            } else {
                rightIntake.setPower(power);
                leftIntake.setPower(power);
            }
    }

    private void sickRobits(double power){
        if (power > maxSpeed){
            rightIntake.setPower(-maxSpeed);
            leftIntake.setPower(-maxSpeed);
        } else {
            rightIntake.setPower(-power);
            leftIntake.setPower(-power);
        }
    }

    @Override
    protected void logTelemetry(Telemetry telemetry) {

    }
}
