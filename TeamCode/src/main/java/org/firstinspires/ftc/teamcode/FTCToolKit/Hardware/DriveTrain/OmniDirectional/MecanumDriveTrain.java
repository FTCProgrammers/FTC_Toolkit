package org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.OmniDirectional;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FTCToolKit.Hardware.DriveTrain.Sensor;
import org.firstinspires.ftc.teamcode.FTCToolKit.Utilities.Toggle;
import static java.lang.Math.*;
public class MecanumDriveTrain extends OmniDirectionalDriveTrain {
    private Toggle changeControl = new Toggle();

    public MecanumDriveTrain(int encoderTicks, double wheelDiameter, Sensor sensor) {
        super(encoderTicks, wheelDiameter, sensor);
    }

    public MecanumDriveTrain(){
        super();
        sensor = Sensor.NONE;
    }

    public MecanumDriveTrain(double wheelDiameter){
        super(wheelDiameter);
        sensor = Sensor.NONE;
    }

    @Override
    public void turn(double power, int degrees) throws InterruptedException {
        double heading = getHeading();
        double newHeading = getHeading() + degrees;
        while(heading != newHeading){
            drive(0,0,-power);
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
    public void turn(double power, double angle) {

    }

    @Override
    public void driveControlled(Gamepad gamepad) {
        if (gamepad.x){
            changeControl.toggle();
        }
        if (changeControl.isToggled()){
            dualStickDrive(gamepad.left_stick_x,-gamepad.left_stick_y,-gamepad.left_stick_y);
        } else {
            drive(gamepad.left_stick_x,gamepad.left_stick_y,gamepad.right_stick_x);
        }
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


    public void encoderDrive(double speed, int distance, double angle, Telemetry telemetry) throws InterruptedException {
        stop();
        resetEncoders();

        int position = (int) (encoderticks * (abs(distance) / wheelCircumference));

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfront.setTargetPosition((int) (-position * signum(sin(angle))));
        leftback.setTargetPosition((int) (-position * signum(sin(angle))));
        rightfront.setTargetPosition((int) (position * signum(sin(angle))));
        rightback.setTargetPosition((int) (position * signum(sin(angle))));

        mecanumMoveAndTurn(speed, angle, 0);

        while(isBusy()) {
            idle();
            logTelemetry(telemetry);
            idle();
        }

        stop();
        resetEncoders();
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


    private void mecanumMoveAndTurn(double movePower, double angle, double turnPower) {
        double x = cos(toRadians(angle)) * movePower;
        double y = sin(toRadians(angle)) * movePower;
        drive(x,y,turnPower);
    }

    @Override
    public void driveByTime(double speed, int seconds, Direction direction, Telemetry telemetry) {
        runtime.reset();
        double x = 0.0, y = 0.0, z = 0.0;
        switch (direction){
            case RIGHT:
                x = speed;
                break;
            case LEFT:
                x = -speed;
                break;
            case FORWARDS:
                y = speed;
                break;
            case BACKWARDS:
                y = -speed;
                break;
        }
        while (runtime.seconds() < seconds){
            drive(x,y,z);
        }
    }

    @Override
    public void encoderDrive(double speed, int distance, Direction direction, Telemetry telemetry) throws InterruptedException {
        stop();
        int target = (int) (encoderticks * (abs(distance) / wheelCircumference));
        switch (direction) {
            case FORWARDS:
                resetEncoders();
                idle();
                leftfront.setTargetPosition(-target);
                rightback.setTargetPosition(target);
                leftback.setTargetPosition(-target);
                rightfront.setTargetPosition(target);
                drive(0,speed,0);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    idle();
                    logTelemetry(telemetry);
                    idle();
                }
                break;
            case BACKWARDS:
                resetEncoders();
                idle();
                leftfront.setTargetPosition(-target);
                rightback.setTargetPosition(-target);
                leftback.setTargetPosition(-target);
                rightfront.setTargetPosition(-target);
                drive(0,-speed,0);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    idle();
                    logTelemetry(telemetry);
                    idle();
                }
                break;
            case RIGHT:
                resetEncoders();
                leftback.setTargetPosition(-target);
                rightback.setTargetPosition(target);
                leftfront.setTargetPosition(target);
                rightfront.setTargetPosition(-target);
                idle();
                drive(speed,0,0);
                idle();
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    idle();
                    logTelemetry(telemetry);
                    idle();
                }
                break;
            case LEFT:
                resetEncoders();
                leftback.setTargetPosition(target);
                rightback.setTargetPosition(-target);
                leftfront.setTargetPosition(-target);
                rightfront.setTargetPosition(target);
                idle();
                drive(-speed,0,0);
                idle();
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (isBusy()) {
                    idle();
                    logTelemetry(telemetry);
                    idle();
                }
                break;
        }
    }

    /**
     * @param x This is the Strafing Value
     * @param y This is the Movement Value
     * @param z This is the Rotation Value
     */
    private void drive(double x, double y, double z) {
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            leftfront.setPower(speedmultiplier*(-y - x - z));
            leftback.setPower(speedmultiplier*(-y + x - z));
            rightfront.setPower(speedmultiplier*(y - x - z));
            rightback.setPower(speedmultiplier*(y + x - z));
        }
    }

    /**
     * @param x This is the strafing value
     * @param y This is the left side wheel value
     * @param z This is the right side wheel value
     */
    private void dualStickDrive(double x, double y, double z) {
        //Another option for Mecanum Driving where Y controls the left stick and Z controls the right stick
        if(abs(x) < 0.01 && abs(y) < 0.01 && abs(z) < 0.01) {
            stop();
        } else {
            leftfront.setPower(speedmultiplier*(y+x));
            leftback.setPower(speedmultiplier*(y-x));
            rightfront.setPower(speedmultiplier*(z-x));
            rightback.setPower(speedmultiplier*(z+x));
        }
    }

    @Override
    public void logTelemetry(Telemetry telemetry) {
        super.logTelemetry(telemetry);
    }
}
