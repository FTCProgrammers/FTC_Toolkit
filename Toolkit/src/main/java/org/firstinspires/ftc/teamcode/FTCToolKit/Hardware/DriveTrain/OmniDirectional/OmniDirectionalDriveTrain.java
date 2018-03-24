package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.DriveTrain;

public abstract class OmniDirectionalDriveTrain extends DriveTrain {
    DcMotor leftfront,rightfront,rightback,leftback;
    BNO055IMU imu;

    public OmniDirectionalDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    public void init(HardwareMap hwMap){
        leftfront = hwMap.dcMotor.get("lf");
        rightfront = hwMap.dcMotor.get("rf");
        leftback = hwMap.dcMotor.get("lb");
        rightback = hwMap.dcMotor.get("rb");
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
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public abstract void drive(double x, double y, double z);
    public abstract void rotate(double z);

}
