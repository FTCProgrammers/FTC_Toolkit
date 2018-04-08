package org.firstinspires.ftc.toolkit.Hardware.DriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.toolkit.Utilities.Sensor;
import static java.lang.Math.*;
public class TwoWheeledDriveTrain extends DriveTrain {
    private DcMotor leftMotor, rightMotor;
    private BNO055IMU imu;
    private GyroSensor gyro;
    private Sensor sensor;
    public TwoWheeledDriveTrain(int encoderTicks, double wheelDiameter, Sensor sensor) {
        super(encoderTicks, wheelDiameter);
        this.sensor = sensor;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftMotor = hwMap.dcMotor.get("left");
        rightMotor = hwMap.dcMotor.get("right");
        if (sensor == Sensor.REV) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            //This Initializes the Parameters for the IMU
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
        } else if (sensor == Sensor.MR){
            gyro = hwMap.gyroSensor.get("gyro");
            gyro.calibrate();
        } else {
            gyro = null;
            imu = null;
        }
    }

    private void drive(double rightpower, double leftpower){
        if (abs(rightpower) > 0.1 && abs(leftpower) > 0.01 ){
            stop();
        } else {
            rightMotor.setPower(speedmultiplier*rightpower);
            leftMotor.setPower(speedmultiplier*leftpower);
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
            heading = 0;
        }
        return heading;
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        if (isBusy()){
            telemetry.addData("Right Encoder Position", rightMotor.getCurrentPosition());
            telemetry.addData("Left Encoder Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.update();
        } else {
            telemetry.addData("Right Power", rightMotor.getPower());
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.update();
        }
    }

    /**
     * @param angle
     * this is the heading that you what the gyro to turn to
     * NOTE: This does not mean it will turn that angle, however it will turn until it reaches that angle as its heading
     */
    @Override
    public void rotate(double power, double angle) throws InterruptedException {
        double heading = getHeading();
        while(angle != heading){
            idle();
            drive(power,-power);
            idle();
        }
    }

    @Override
    public void turn(double power, int degrees){
        double newHeading = getHeading() + degrees;
        double startHeading = getHeading();
        while (startHeading != newHeading){
            drive(power,-power);
        }
    }

    @Override
    public void driveControlled(Gamepad gamepad) {

    }

    @Override
    public void stop() {
        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);
    }

    @Override
    public boolean isBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    @Override
    public void setMotorPower(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) throws InterruptedException {
        switch (direction){
            case FORWARDS:
                runtime.reset();
                while (runtime.seconds() < seconds){
                    drive(speed,speed);
                }
                break;
            case BACKWARDS:
                runtime.reset();
                while (runtime.seconds() < seconds){
                    drive(-speed,-speed);
                }
                break;
        }
    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) throws InterruptedException {
        resetEncoders();
        int target = (int) (encoderticks * (abs(distance) / wheelCircumference));
        switch (direction){
            case FORWARDS:
                idle();
                setTargetPosition(target);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive(speed,speed);
                while(isBusy()){
                    logTelemetry(telemetry);
                }
                break;
            case BACKWARDS:
                idle();
                setTargetPosition(-target);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive(-speed,-speed);
                while(isBusy()){
                    logTelemetry(telemetry);
                }
                break;
        }
        resetEncoders();
    }

    @Override
    public void setTargetPosition(int targetPosition) {
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }
}
