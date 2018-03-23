package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

//This class has code for Holonomic Drivetrains
public class HolonomicDriveTrain extends OmniDirectionalDriveTrain {

    public HolonomicDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    @Override
    public void turn(double power, double angle) {

    }

    @Override
    public void driveControlled(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double z = gamepad.right_stick_x;
        drive(x,y,z);
    }

    public void rotate(double z) {
        drive(0.0, 0.0, z);
    }

    private void encoderTelemetry(Telemetry telemetry) throws InterruptedException {
        telemetry.addData("LB Position", leftback.getCurrentPosition());
        telemetry.addData("LF Position", leftfront.getCurrentPosition());
        telemetry.addData("RB Position", rightback.getCurrentPosition());
        telemetry.addData("RF Position", rightfront.getCurrentPosition());
        telemetry.addData("Heading", getHeading());
        telemetry.update();
    }

    @Override
    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        stop();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double heading(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    public double getHeading(){
        //this makes it so the heading range is from 0-360 instead of -180-180
        double heading = heading();
        if (heading < 0) {
            heading += 360;
        }
        return heading;
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

    public void fieldCentricDrive(double x, double y, double z){
        double theta = getHeading();
        double X = speedmultiplier*(x*cos(theta) - y*sin(theta));
        double Y = speedmultiplier*(x*sin(theta) + y*cos(theta));
        double Z = speedmultiplier*z; //I did this so it looks less weird
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            //This makes it so one stick powers the whole drivetrain and the other powers rotation
            leftfront.setPower(scalePower(Y + X + z));
            rightfront.setPower(scalePower(Y - X - z));
            leftback.setPower(scalePower(Y - X + z));
            rightback.setPower(scalePower(Y + X - Z));
        }
    }

    public void drive(double x, double y, double z){
        x*=speedmultiplier;
        y*=speedmultiplier;
        z*=speedmultiplier;
        //This allows
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            //This makes it so one stick powers the whole drivetrain and the other powers rotation
            leftfront.setPower(scalePower(y + x + z));
            rightfront.setPower(scalePower(y - x - z));
            leftback.setPower(scalePower(y - x + z));
            rightback.setPower(scalePower(y + x - z));
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
        telemetry.addLine("Drivetrain debugging");
        // display encoder positions if it's enabled
        if(leftfront.getMode() == DcMotor.RunMode.RUN_TO_POSITION || leftfront.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            try {
                encoderTelemetry(telemetry);
            } catch (InterruptedException e) {
                telemetry.addLine("Error with Encoders");
            }
        }
        // display motor powers
        telemetry.addData("Left Front Power", leftfront.getPower());
        telemetry.addData("Right Front Power", rightfront.getPower());
        telemetry.addData("Left Back Power", leftback.getPower());
        telemetry.addData("Right Back Power", rightback.getPower());
        telemetry.update();
    }
}
