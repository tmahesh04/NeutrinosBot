package org.firstinspires.ftc.robotcontroller.external.samples;





import static android.os.Build.ID;



/**

 * These are the constants used in PID

 */

public interface PID_Constants {
    double
    //KP_TURN = 0.0083,
            KP_TURN = 0.003,
    KI_TURN = 0.0002,
    //KD_TURN = 0.0006,
    KD_TURN = 0,
    ID = 1;

    double

            KP_STRAIGHT = 0,

    KI_STRAIGHT = 0,
    KD_STRAIGHT = 0;
    double CLICKS_PER_INCH = 1120 / 4 / Math.PI;

    double
            P_GAIN = 1,

    DEFAULT_TURN_SPEED = .5,

    DEFAULT_TURN_TIMEOUT = 4.0,

    DEFAULT_STRAIGHT_SPEED = 0.5,

    DEFAULT_STRAIGHT_TIMEOUT = 8.0;

    double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    double     DEFAULT_SLEEP_TIME      = 1000.0;
}

