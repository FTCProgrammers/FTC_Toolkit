package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.HardwareComponents.*;

import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Sensor;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.HardwareComponents.Drivetrains.*;

//This class has code for Holonomic Drivetrains
public class HolonomicDriveTrain extends OmniDirectionalDriveTrain {
    /**
     * Gives drive train the values it needs to calculate how to properly apply motor powers
     * when moving and turning autonomously
     * @param wheelDiameter The diameter of the robot's wheels; unit is unnecessary as long as it is consistent with other distances
     * @param encoderTicks The number of ticks given off by each motor's
     *                                encoder each rotation.
     */
    private Drivetrains drivetrain = HOLONOMIC;
    public HolonomicDriveTrain(int encoderTicks, double wheelDiameter, Sensor sensor) {
        super(encoderTicks, wheelDiameter, sensor);
    }

    @Override
    public void rotate(double power, double angle) throws InterruptedException {
        stop();
        double startHeading = getHeading();
        double currentHeading = getHeading() - startHeading;
        while(abs(currentHeading - angle) > 1) { //1 degree error margin
            idle();
            currentHeading = getHeading() - startHeading;
            drive(0.0,0.0,-power * (Math.signum(angle - currentHeading)));
            idle();
        }
        stop();
    }

    @Override
    public void driveControlled(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double z = gamepad.right_stick_x;
        drive(x,y,z);
    }

    @Override
    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        stop();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void turn(double power, int degrees) throws InterruptedException {
        double startHeading = getHeading();
        double targetHeading = startHeading + degrees;
        while (abs(targetHeading - startHeading) > 1){
         drive(0.0,0.0,-power * (signum(targetHeading - startHeading)));
        }
    }

    @Override
    public double getHeading() {
        double heading;
        if (sensor == Sensor.MR){
            heading = gyro.getHeading();
        } else if (sensor == Sensor.REV){
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = orientation.firstAngle;
            if (heading < 0){
                heading+=360;
            }
        } else {
            heading = 1;
        }
        return heading;
    }

    public void move(double speed, int distance, double angle, Telemetry telemetry) {

    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) {

    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) {

    }

    public void fieldCentricDrive(double x, double y, double z){
        double theta = getHeading();
        double X = speedmultiplier*(x*cos(theta) - y*sin(theta));
        double Y = speedmultiplier*(x*sin(theta) + y*cos(theta));
        double Z = speedmultiplier*z;
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            //This makes it so one stick powers the whole drivetrain and the other powers rotation
            leftfront.setPower(-Y - X - Z);
            rightfront.setPower(Y - X - Z);
            leftback.setPower(-Y + X - Z);
            rightback.setPower(Y + X - Z);
        }
    }

    public void drive(double x, double y, double z){
        x*=speedmultiplier;
        y*=speedmultiplier;
        z*=speedmultiplier;
        //This allows the speed multiplier to work
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            //This makes it so one stick powers the whole drivetrain and the other powers rotation
            leftfront.setPower(-y - x - z);
            rightfront.setPower(y - x - z);
            leftback.setPower(-y + x - z);
            rightback.setPower(y + x - z);
        }
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


    public void setMotorPower(double power) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        leftback.setPower(power);
        rightback.setPower(power);
    }


    public void setMode(DcMotor.RunMode runMode) {
        leftfront.setMode(runMode);
        rightfront.setMode(runMode);
        leftback.setMode(runMode);
        rightback.setMode(runMode);
    }

    @Override
    public boolean isBusy() {
        return leftback.isBusy() || leftfront.isBusy() || rightfront.isBusy() || rightback.isBusy();
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        super.logTelemetry(telemetry);
    }

}
