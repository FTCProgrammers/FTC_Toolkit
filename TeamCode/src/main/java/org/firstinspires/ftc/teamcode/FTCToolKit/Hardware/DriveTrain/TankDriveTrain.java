package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Sensor;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Toggle;
import static java.lang.Math.*;

public class TankDriveTrain extends DriveTrain {
    private DcMotor rightfront,leftfront,rightback,leftback;
    private BNO055IMU imu;
    private GyroSensor gyro;
    private Toggle preciseToggle = new Toggle();
    private Toggle newFrontToggle = new Toggle(0.3);
    private Sensor sensor;
    public TankDriveTrain(int encoderTicks, double wheelDiameter, Sensor sensor) {
        super(encoderTicks, wheelDiameter);
        this.sensor = sensor;
    }
    public TankDriveTrain(){
        super();
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftfront = hwMap.dcMotor.get("lf");
        rightfront = hwMap.dcMotor.get("rf");
        leftback = hwMap.dcMotor.get("lb");
        rightback = hwMap.dcMotor.get("rb");
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
        }
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        stop();
    }

    private void drive(double rightpower, double leftpower){
        if (abs(rightpower) < 0.1 && abs(leftpower) < 0.01){
            stop();
        } else {
            leftfront.setPower(speedmultiplier*leftpower);
            rightfront.setPower(speedmultiplier*rightpower);
            leftback.setPower(speedmultiplier*leftpower);
            rightback.setPower(speedmultiplier*rightpower);
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
    public void turn(double power, int degrees) throws InterruptedException {
        double newHeading = getHeading() + degrees;
        double heading = getHeading();
        idle();
        while(heading != newHeading){
            idle();
            drive(-power,power);
            idle();
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

    @Override
    public void driveControlled(Gamepad gamepad) {
        setDefaultSpeed();
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

    @Override
    public void setMotorPower(double power) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        leftback.setPower(power);
        rightback.setPower(power);
    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) {
        runtime.reset();
        switch (direction){
            case FORWARDS:
                while (runtime.seconds() < seconds){
                    drive(speed,speed);
                }
                break;

            case BACKWARDS:
                while (runtime.seconds() < seconds){
                    drive(-speed,-speed);
                }
                break;
        }
    }

    @Override
    public void turn(double power, double angle) throws InterruptedException {
        double heading = getHeading();
        idle();
        while(angle != heading){
            idle();
            drive(-power,power);
            idle();
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
            telemetry.addData("LB Position", leftback.getCurrentPosition());
            telemetry.addData("LF Position", leftfront.getCurrentPosition());
            telemetry.addData("RB Position", rightback.getCurrentPosition());
            telemetry.addData("RF Position", rightfront.getCurrentPosition());
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
