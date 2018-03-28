package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.Sensor;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Constants;

public abstract class OmniDirectionalDriveTrain extends DriveTrain {
    Sensor sensor;
    DcMotor leftfront, rightfront, rightback, leftback;
    BNO055IMU imu;
    GyroSensor gyro;

    protected OmniDirectionalDriveTrain(int encoderTicks, double wheelDiameter, Sensor sensor) {
        super(encoderTicks, wheelDiameter);
        this.sensor = sensor;
    }

    protected OmniDirectionalDriveTrain(){
        super();
    }

    protected OmniDirectionalDriveTrain(double wheelDiameter){
        super(wheelDiameter);
        encoderticks = Constants.ANDYMARK_MOTOR_TICKS;
    }

    public void logTelemetry(Telemetry telemetry){
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
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        } else {
            telemetry.addData("Left Front Power", leftfront.getPower());
            telemetry.addData("Right Front Power", rightfront.getPower());
            telemetry.addData("Left Back Power", leftback.getPower());
            telemetry.addData("Right Back Power", rightback.getPower());
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
    }
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
        } else {
            gyro = null;
            imu = null;
        }
    }
}