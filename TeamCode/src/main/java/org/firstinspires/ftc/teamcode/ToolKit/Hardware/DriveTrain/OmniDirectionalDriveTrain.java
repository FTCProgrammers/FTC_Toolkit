package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class OmniDirectionalDriveTrain extends DriveTrain {
    public DcMotor leftfront,rightfront,rightback,leftback;

    public OmniDirectionalDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    public void init(HardwareMap hwMap){
        leftfront = hwMap.dcMotor.get("lf");
        rightfront = hwMap.dcMotor.get("rf");
        leftback = hwMap.dcMotor.get("lb");
        rightback = hwMap.dcMotor.get("rb");
    }
    public abstract void drive(double x, double y, double z);
    public abstract void rotate(double z);
    public abstract void setTargetPosition(int targetPosition);
    public abstract void encoderDrive(double speed, int distance, double angle, Telemetry telemetry);
    public abstract void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry);
    public abstract void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry);
}
