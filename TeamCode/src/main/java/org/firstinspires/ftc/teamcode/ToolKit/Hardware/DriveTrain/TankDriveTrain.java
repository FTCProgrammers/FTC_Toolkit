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
import org.firstinspires.ftc.teamcode.ToolKit.Utilities.Toggle;
import static java.lang.Math.*;
/**
 * Created by shaunaksarker on 3/19/18.
 */

public class TankDriveTrain extends DriveTrain {
    private DcMotor rightfront,leftfront,rightback,leftback;
    private BNO055IMU imu;
    private Toggle preciseToggle = new Toggle();
    private Toggle newFrontToggle = new Toggle();
    public TankDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }
    @Override
    public void init(HardwareMap hwMap) {
        leftfront = hwMap.dcMotor.get("lf");
        rightfront = hwMap.dcMotor.get("rf");
        leftback = hwMap.dcMotor.get("lb");
        rightback = hwMap.dcMotor.get("rb");
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParamInit();
        stop();
    }

    private void drive(double rightpower, double leftpower){
        if (abs(rightpower) < 0.1 && abs(leftpower) < 0.01){
            stop();
        } else {
            leftfront.setPower(scalePower(speedmultiplier*leftpower));
            rightfront.setPower(scalePower(speedmultiplier*rightpower));
            leftback.setPower(scalePower(speedmultiplier*leftpower));
            rightback.setPower(scalePower(speedmultiplier*rightpower));
        }
    }

    private void encoderTelemetry(Telemetry telemetry){
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

    private double heading(){
        //IMU gives a heading between -180 and 180
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

    private void imuParamInit() {
        //This Initializes the Parameters for the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    @Override
    public void driveControlled(Gamepad gamepad) {
        if (gamepad.x)
            newFrontToggle.toggle();

        if (gamepad.y){
            preciseToggle.toggle();
        }

        if (preciseToggle.isToggled()){
            speedmultiplier = 0.5;
        }
        //For Tank Drive This Allows You to Dictate the Front Of The Robot
        if (newFrontToggle.isToggled()){
            double leftpower = gamepad.left_stick_y;
            double rightpower = gamepad.right_stick_y;
            drive(rightpower, leftpower);
        } else {
            double leftpower = -gamepad.left_stick_y;
            double rightpower = -gamepad.right_stick_y;
            drive(rightpower, leftpower);
        }

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

    public void turn(double power, double angle) throws InterruptedException {
        double heading = getHeading();
        idle();
        if (angle > heading){
            idle();
            while(angle != heading){
                idle();
                drive(-power,power);
            }
        } else if (angle < heading){
            idle();
            while (angle != heading){
                idle();
                drive(power,-power);
            }
        }
    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) {
        //Tank Cannot Strafe so Directions LEFT and RIGHT will lead to nothing
        resetEncoders();
        int target = (int) (encoderticks * (abs(distance) / wheelCircumference));
        switch (direction){
            case FORWARDS:
                setTargetPosition(target);
                setMotorPower(speed);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    logTelemetry(telemetry);
                }
                break;
            case BACKWARDS:
                setTargetPosition(-target);
                setMotorPower(speed);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                   logTelemetry(telemetry);
                }
                break;
        }
    }

    @Override
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
