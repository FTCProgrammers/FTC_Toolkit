package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCToolKit.Robot;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Constants;
public abstract class DriveTrain extends Robot {
    protected double speedmultiplier = 1.0;
    protected ElapsedTime runtime = new ElapsedTime();
    protected int encoderticks;
    protected final double wheelCircumference;
    public void setDefaultSpeed(){
        speedmultiplier = 1.0;
    }
    public void changeSpeed(double speed){
        speedmultiplier = speed;
    }

    public DriveTrain(int encoderTicks, double wheelDiameter){
        this.encoderticks = encoderTicks;
        wheelCircumference = Math.PI * wheelDiameter;
    }

    public DriveTrain(double wheelDiameter){
        encoderticks = Constants.ANDYMARK_MOTOR_TICKS;
        wheelCircumference = Math.PI * wheelDiameter;
    }

    public DriveTrain(){
        encoderticks = Constants.ANDYMARK_MOTOR_TICKS;
        wheelCircumference = Math.PI * Constants.inWheelDiameter;
    }

    public enum Direction {
        FORWARDS,BACKWARDS,LEFT,RIGHT
    }

    protected void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public abstract void turn(double power, int degrees) throws InterruptedException;
    public abstract double getHeading();
    public abstract void rotate(double power, double angle) throws InterruptedException;
    public abstract void driveControlled(Gamepad gamepad);
    public abstract void stop();
    public abstract boolean isBusy();
    public abstract void setMotorPower(double power);
    public abstract void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) throws InterruptedException;
    public abstract void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) throws InterruptedException;
    public abstract void setTargetPosition(int targetPosition);
    public abstract void setMode(DcMotor.RunMode runMode);
}
