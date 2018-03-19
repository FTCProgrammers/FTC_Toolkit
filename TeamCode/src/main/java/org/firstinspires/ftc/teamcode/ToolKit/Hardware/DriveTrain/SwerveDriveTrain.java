package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by shaunaksarker on 3/19/18.
 */
//This Class is nowhere near finished, it needs to be worked on
public class SwerveDriveTrain extends OmniDirectionalDriveTrain {
    double L = 1, W = 2; //Finish this with actual robot length and width
    public SwerveDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {

    }

    @Override
    public void drive(double x, double y, double z) {
        double r = Math.sqrt ((L * L) + (W * W));
        y *= -1;

        double a = x - z * (L / r);
        double b = x + z * (L / r);
        double c = y - z * (W / r);
        double d = y + z * (W / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

    }

    @Override
    public void rotate(double z) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void setTargetPosition(int targetPosition) {

    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setMotorPower(double power) {

    }

    @Override
    public void setZeroPowerBehavior() {

    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {

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
}
