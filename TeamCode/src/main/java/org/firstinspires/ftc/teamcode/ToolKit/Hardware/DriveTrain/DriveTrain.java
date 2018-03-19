package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ToolKit.Robot;

import static java.lang.Math.PI;

public abstract class DriveTrain extends Robot {
    public double speedmultiplier = 1.0;
    public ElapsedTime runtime = new ElapsedTime();
    public int encoderticks;
    public void setDefaultSpeed(){
        speedmultiplier = 1.0;
    }
    protected double wheelCircumference;

    public DriveTrain(int encoderTicks, double wheelDiameter){
        this.encoderticks = encoderTicks;
        double wheelCircumference = Math.PI * wheelDiameter;
    }

    public enum Direction{
        FORWARDS,BACKWARDS,LEFT,RIGHT
    }

    public double scalePower(double power) {
        //This Is The Power Scaling Method
        float scaledPower;
        power = Range.clip(power, -1, 1);
        float[] possiblePowerValues = {
                0.00f, 0.05f, 0.09f, 0.10f, 0.12f,
                0.15f, 0.18f, 0.24f, 0.30f, 0.36f,
                0.43f, 0.50f, 0.60f, 0.72f, 0.85f,
                1.00f, 1.00f
        };
        int powerIndex = (int) (power * 16.0);
        if (powerIndex < 0) {
            powerIndex = -powerIndex;
        } else if (powerIndex > 16) {
            powerIndex = 16;
        }
        if (power < 0) {
            scaledPower = -possiblePowerValues[powerIndex];
        } else {
            scaledPower = possiblePowerValues[powerIndex];
        }
        return scaledPower;
    }

    public void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public abstract void stop();
    public abstract boolean isBusy();
    public abstract void setMotorPower(double power);
    public abstract void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry);
    public abstract void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry);
    public abstract void setTargetPosition(int targetPosition);
    public abstract void setMode(DcMotor.RunMode runMode);
}
