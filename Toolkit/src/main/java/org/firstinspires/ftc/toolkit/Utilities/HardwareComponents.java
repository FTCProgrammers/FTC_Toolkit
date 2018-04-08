package org.firstinspires.ftc.toolkit.Utilities;
public interface HardwareComponents {

    enum Drivetrains {
        MECANUM, HOLONOMIC, TANK, TWOWHEELED
    }

    enum Sensors{
        REV,MR,NONE
    }

    enum Motors {
        LEFT_FRONT,LEFT_BACK,RIGHT_FRONT,RIGHT_BACK
    }

    enum GrabSystem{
        SERVO, WHEELEDINTAKE, ELSE
    }

    Drivetrains getDrivetrain();
}


