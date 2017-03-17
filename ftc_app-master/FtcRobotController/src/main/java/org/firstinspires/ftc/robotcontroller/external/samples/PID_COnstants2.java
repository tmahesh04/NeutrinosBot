package org.firstinspires.ftc.robotcontroller.external.samples;



/**

 * This interface contains various constants for using PID control in autonomous.

 */



public interface PID_COnstants2 {



    double

            KP_TURN = 0.01,

    KI_TURN = 0.0,

    KD_TURN = 0.0,

    I_DAMPER_TURN = 1.0;



    double

            P_GAIN = 1.0/12.0,

    KP_STRAIGHT = 0.1,

    KI_STRAIGHT = 0.0,

    KD_STRAIGHT = 0.05,

    I_DAMPER_STRAIGHT = 1.0;



    double

            DEFAULT_TURN_SPEED = .5,

    DEFAULT_TURN_TIMEOUT = 4.0,

    DEFAULT_STRAIGHT_SPEED = 0.5,

    DEFAULT_STRAIGHT_TIMEOUT = 8.0;



    double CLICKS_PER_INCH = 1120 / 4 / Math.PI; // change this to be dependent on robot



}