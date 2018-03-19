package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static java.lang.Math.abs;

//This class has code for Holonomic Drivetrains

public class HolonomicDriveTrain extends OmniDirectionalDriveTrain{

    private BNO055IMU imu;

    public HolonomicDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    public void rotate(double z) {
        //-1 for z makes it turn to the right
        //1 for z makes it turn to left
        double y = 0.0,x = 0.0;
        drive(x, y, z);
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
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParamInit();
        stop();
    }

    public double heading(){
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
    public void encoderDrive(double speed, int distance, double angle, Telemetry telemetry) {

    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) {

    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) {

    }

    public void drive(double x, double y, double z){
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            //This makes it so one stick powers the whole drivetrain and the other powers rotation
            leftfront.setPower(scalePower(speedmultiplier * (y + x + z)));
            rightfront.setPower(scalePower(speedmultiplier * (y - x - z)));
            leftback.setPower(scalePower(speedmultiplier * (y - x + z)));
            rightback.setPower(scalePower(speedmultiplier * (y + x - z)));
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
