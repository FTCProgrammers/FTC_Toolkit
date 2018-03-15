package org.firstinspires.ftc.teamcode.ToolKit.Hardware.Sensors;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RevSensorClass extends Sensors {

    public void init(HardwareMap hwMap) {
        sensorColor = hwMap.get(ColorSensor.class, "cds");
        sensorDistance = hwMap.get(DistanceSensor.class, "cds");
        imuParamInit();
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        if (readColor() == Color.BLUE) {
            telemetry.addLine("BLUE");
            telemetry.addData("Distance", readDistance());
        } else if (readColor() == Color.BLUE) {
            telemetry.addLine("RED");
            telemetry.addData("Distance", readDistance());}
    }


    public Color readColor() {
        final int TOLERANCE = 12;
        boolean blueJewel = false, redJewel = false;
        if(sensorColor.blue() - sensorColor.red() >= TOLERANCE) {
            blueJewel = true;
            redJewel = false;
        } else if(sensorColor.red() - sensorColor.blue() >= TOLERANCE) {
            redJewel = true;
            blueJewel = false;
        }
        return blueJewel ? Color.BLUE : (redJewel ? Color.RED : Color.ELSE);
    }

    public double readDistance() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
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

    public double Heading() {
        //this makes it so the heading range is from -180-180
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    public double getHeading() throws InterruptedException {
        //this makes it so the heading range is from 0-360 instead of -180-180
        double heading = Heading();
        idle();
        if (heading < 0) {
            idle();
            heading += 360;
            idle();
        }
        idle();
        return heading;
    }


}
