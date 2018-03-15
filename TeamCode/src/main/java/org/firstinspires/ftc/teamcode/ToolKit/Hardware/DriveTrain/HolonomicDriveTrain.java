package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//This class has code for Holonomic Drivetrains

public class HolonomicDriveTrain extends OmniDirectionalDriveTrain{
    public BNO055IMU imu;
    public HolonomicDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    public void rotate(double z) {
        //-1 for z makes it turn to the right
        //1 for z makes it turn to left
        double y = 0.0,x = 0.0;
        drive(x, y, z);
    }

    @Override
    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        setZeroPowerBehavior();
        stop();
    }

    @Override
    public void encoderDrive(double speed, int distance, double angle, Telemetry telemetry) {

    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) {

    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) {

    }

    public void drive(double x, double y, double z){
        //This makes it so one stick powers the whole drivetrain and the other powers rotation
        leftfront.setPower(scalePower(speedmultiplier * (y + x + z)));
        rightfront.setPower(scalePower(speedmultiplier * (y - x - z)));
        leftback.setPower(scalePower(speedmultiplier * (y - x + z)));
        rightback.setPower(scalePower(speedmultiplier * (y + x - z)));
    }

    @Override
    public void stop() {
        drive(0,0,0);
    }

    @Override
    public void setTargetPosition(int target) {
        leftfront.setTargetPosition(target);
        rightfront.setTargetPosition(target);
        leftback.setTargetPosition(target);
        rightback.setTargetPosition(target);
    }


    public void setMotorPower(double power) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        leftback.setPower(power);
        rightback.setPower(power);
    }

    public void setZeroPowerBehavior() {
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(DcMotor.RunMode runMode) {
        leftfront.setMode(runMode);
        rightfront.setMode(runMode);
        leftback.setMode(runMode);
        rightback.setMode(runMode);
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {

    }
}
