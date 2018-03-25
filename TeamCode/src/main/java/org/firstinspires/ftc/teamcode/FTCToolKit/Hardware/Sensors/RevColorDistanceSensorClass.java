package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.Sensors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTCToolKit.Robot;

public class RevColorDistanceSensorClass extends Robot{
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public enum Color {
        BLUE,RED,ELSE
    }

    public void init(HardwareMap hwMap) {
        sensorColor = hwMap.get(ColorSensor.class, "cds");
        sensorDistance = hwMap.get(DistanceSensor.class, "cds");
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

}
