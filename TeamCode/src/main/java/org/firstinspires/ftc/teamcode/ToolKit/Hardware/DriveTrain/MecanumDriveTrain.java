package org.firstinspires.ftc.teamcode.ToolKit.Hardware.DriveTrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static java.lang.Math.*;

public class MecanumDriveTrain extends OmniDirectionalDriveTrain {
    public MecanumDriveTrain(int encoderTicks, double wheelDiameter) {
        super(encoderTicks, wheelDiameter);
    }

    @Override
    public void turn(double power, double angle) {

    }

    @Override
    public void driveControlled(Gamepad gamepad) {

    }
    @Override
    public void stop() {
        setMotorPower(0.0);
    }

    @Override
    public void setTargetPosition(int target) {
        leftfront.setTargetPosition(target);
        rightfront.setTargetPosition(target);
        leftback.setTargetPosition(target);
        rightback.setTargetPosition(target);
    }

    @Override
    public void setMotorPower(double power) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        leftback.setPower(power);
        rightback.setPower(power);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        leftfront.setMode(runMode);
        rightfront.setMode(runMode);
        leftback.setMode(runMode);
        rightback.setMode(runMode);
    }

    @Override
    public void rotate(double z) {
        double x = 0.0, y = 0.0;
        drive(x,y,z);
    }

    @Override
    public void init(HardwareMap hwMap) {
        super.init(hwMap);
        stop();
    }

    @Override
    public boolean isBusy() {
        return leftback.isBusy() || leftfront.isBusy() || rightfront.isBusy() || rightback.isBusy();
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

    /**
     * @param x This is the Strafing Value
     * @param y This is the Movement Value
     * @param z This is the Rotation Value
     */
    @Override
    public void drive(double x, double y, double z) {
        x*=speedmultiplier;
        y*=speedmultiplier;
        z*=speedmultiplier;
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            leftfront.setPower(scalePower(-y - x - z));
            leftback.setPower(scalePower(-y + x - z));
            rightfront.setPower(scalePower(y - x - z));
            rightback.setPower(scalePower(y + x - z));
        }
    }

    public void dualStickDrive(double x, double y, double z) {
        //Another option for Mecanum Driving
        x*=speedmultiplier;
        y*=speedmultiplier;
        z*=speedmultiplier;
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            leftfront.setPower(scalePower(y+x));
            leftback.setPower(scalePower(y-x));
            rightfront.setPower(scalePower(z-x));
            rightback.setPower(scalePower(z+x));
        }
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {

    }
}
