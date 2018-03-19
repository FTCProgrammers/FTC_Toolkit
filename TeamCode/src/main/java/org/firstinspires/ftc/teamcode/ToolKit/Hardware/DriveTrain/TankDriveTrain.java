package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static java.lang.Math.abs;
/**
 * Created by shaunaksarker on 3/19/18.
 */

public class TankDriveTrain extends DriveTrain {
    private DcMotor rightfront,leftfront,rightback,leftback;
    private BNO055IMU imu;

    public TankDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftfront = hwMap.dcMotor.get("lf");
        rightfront = hwMap.dcMotor.get("rf");
        leftback = hwMap.dcMotor.get("lb");
        rightback = hwMap.dcMotor.get("rb");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        stop();
    }

    public void drive(double rightpower, double leftpower){
        if (abs(rightpower) < 0.1 && abs(leftpower) < 0.01){
            stop();
        } else {
            leftfront.setPower(scalePower(leftpower));
            rightfront.setPower(scalePower(rightpower));
            leftback.setPower(scalePower(leftpower));
            rightback.setPower(scalePower(rightpower));
        }
    }

    public void encoderTelemetry(Telemetry telemetry){
        telemetry.addData("LB Position", leftback.getCurrentPosition());
        telemetry.addData("LF Position", leftfront.getCurrentPosition());
        telemetry.addData("RB Position", rightback.getCurrentPosition());
        telemetry.addData("RF Position", rightfront.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftback.setPower(0);
        leftfront.setPower(0);
        rightback.setPower(0);
        rightfront.setPower(0);
    }

    @Override
    public void setTargetPosition(int target) {
        leftfront.setTargetPosition(target);
        rightfront.setTargetPosition(target);
        leftback.setTargetPosition(target);
        rightback.setTargetPosition(target);
    }

    @Override
    public boolean isBusy() {
        return leftback.isBusy() || leftfront.isBusy() || rightfront.isBusy() || rightback.isBusy();
    }

    public void setMotorPower(double power) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        leftback.setPower(power);
        rightback.setPower(power);
    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) {

    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) {
        //Tank Cannot Strafe so Directions LEFT and RIGHT will lead to nothing
        stop();
        int target = (int) (encoderticks * (abs(distance) / wheelCircumference));
        switch (direction){
            case FORWARDS:
                resetEncoders();
                leftfront.setTargetPosition(target);
                rightback.setTargetPosition(target);
                leftback.setTargetPosition(target);
                rightfront.setTargetPosition(target);
                setMotorPower(speed);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    encoderTelemetry(telemetry);
                }
                break;
            case BACKWARDS:
                resetEncoders();
                leftfront.setTargetPosition(target);
                rightback.setTargetPosition(target);
                leftback.setTargetPosition(target);
                rightfront.setTargetPosition(target);
                setMotorPower(speed);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    encoderTelemetry(telemetry);
                }
                break;
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        leftfront.setMode(runMode);
        rightfront.setMode(runMode);
        leftback.setMode(runMode);
        rightback.setMode(runMode);
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        telemetry.addLine("Drivetrain debugging");
        if (isBusy()) {
            encoderTelemetry(telemetry);
            telemetry.addData("Left Front Power", leftfront.getPower());
            telemetry.addData("Right Front Power", rightfront.getPower());
            telemetry.addData("Left Back Power", leftback.getPower());
            telemetry.addData("Right Back Power", rightback.getPower());
            telemetry.update();
        } else {
            telemetry.addData("Left Front Power", leftfront.getPower());
            telemetry.addData("Right Front Power", rightfront.getPower());
            telemetry.addData("Left Back Power", leftback.getPower());
            telemetry.addData("Right Back Power", rightback.getPower());
            telemetry.update();
        }
    }
}
