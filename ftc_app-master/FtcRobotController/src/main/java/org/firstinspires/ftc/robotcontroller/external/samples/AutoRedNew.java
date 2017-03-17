package org.firstinspires.ftc.robotcontroller.external.samples;

import android.bluetooth.BluetoothA2dp;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "AutoRedNewV8", group = "Test") // change name
public class AutoRedNew extends NeutLinearOpMode { // change file name
    public void main() throws InterruptedException {

    }
    @Override
    public void runLinearOpMode() throws InterruptedException {
        final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;


        //NeutrinosBEBE robot = new NeutrinosBEBE(telemetry);
        while (!isStarted()) {
            telemetry.addLine("hello aneesha");
            telemetry.addData("imu", robot.imu.getHeading());
            telemetry.update();
//            robot.redServo.setDirection(Servo.Direction.REVERSE);
            robot.blueServo.setDirection(Servo.Direction.FORWARD);
            robot.blueServo.setPosition(1);
//            robot.redServo.setPosition(.03);
            robot.armServo.setPosition(0);

        }
        waitForStart();


        robot.drivePID(.5,33,Direction.FORWARD, 100 ,0);
        sleep(100);
        robot.shootBall(1, 1.5);
        robot.armMove();
        robot.shootBall(1, 1.5);
        robot.turnPIDSlow(0.3,45, Direction.LEFT, 2.0, 1000);
        sleep(100);
        robot.drivePID(.7,77,Direction.FORWARD,100, 45);
        robot.touchSensorMoveFront(.3);
        robot.drivePID(.3, 8, Direction.BACKWARD,100,0);
//        robot.turnPIDCustomKPKIKD(.3, 0, Direction.RIGHT, 2.0, 1000, .015, .08,0);
//        robot.colorODSForward(.4,.4);
//        sleep(250);
//        robot.colorODSBackward(.2,.2);
//        robot.drivePID(.3,3,Direction.FORWARD,100,0);
//        robot.colorRedMove();
//        robot.turnPIDSlow(.3, 5, Direction.LEFT, 1.0,0);
//        robot.blueServo.setPosition(1);
//        robot.redServo.setPosition(.03);
//        robot.drivePID(.3, 12, Direction.BACKWARD, 100, 0);
//        robot.colorODSBackward(.4,.4);
//        sleep(250);
//        robot.colorODSForward(.2,.2);
//        robot.drivePID(.3,2,Direction.FORWARD,100,0);
//        robot.colorRedMove();
//        robot.blueServo.setPosition(1);
//        robot.redServo.setPosition(.03);
//        robot.turnPID(.3,10,Direction.LEFT,1.5,100);
//        robot.drivePID(.6,48, Direction.BACKWARD, 100, 20);
//        sleep(3000);
    }

}

