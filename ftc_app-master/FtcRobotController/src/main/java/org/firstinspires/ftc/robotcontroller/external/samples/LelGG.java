//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import android.bluetooth.BluetoothA2dp;
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//import android.graphics.Color;
//import com.qualcomm.ftccommon.SoundPlayer;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
///**
// * Created by Archish on 10/6/16.
// */
//
//@Autonomous(name = "ioehiuhewr", group = "Test") // change name
//public class LelGG extends LinearOpMode { // change file name
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
//        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                (WHEEL_DIAMETER_INCHES * 3.1415);
//        NeutrinosBEBE lol = new NeutrinosBEBE(telemetry);
////        lol.redServo.setDirection(Servo.Direction.REVERSE);
//        lol.blueServo.setDirection(Servo.Direction.FORWARD);
//        lol.blueServo.setPosition(1);
//
//        lol.armServo.setPosition(0);
//
//
//        while (!isStarted()) {
//            lol.addTelemetry("hello aneesha");
//            telemetry.update();
//        }
//        lol.drivePID(.5,33,Direction.FORWARD, 100 ,0);
//        lol.shootBall(1, 1.5);
//        lol.armMove();
//        lol.shootBall(1, 1.5);
//        lol.turnPIDSlow(0.3, 50, Direction.LEFT, 2.0, 1000);
//        lol.drivePID(.7,78, Direction.FORWARD,100,45);
//        sleep(250);
//        lol.touchSensorMoveFront(.3);
//        lol.drivePID(.4,8,Direction.BACKWARD,100, 45);
//        lol.turnPIDSlow(.3, 2, Direction.RIGHT, 2.0, 1000);
//        lol.colorODSForward(.4,.4);
//        sleep(250);
//        lol.colorODSBackward(.2,.2);
//        lol.drivePID(.3,3,Direction.FORWARD,100,0);
//        lol.colorRedMove();
//        lol.turnPIDSlow(.3, 5, Direction.LEFT, 1.0,0);
//        lol.blueServo.setPosition(1);
////        lol.redServo.setPosition(.03);
//        lol.turnPIDSlow(.3, 5, Direction.RIGHT, 1.0,0);
//        lol.drivePID(.3, 12, Direction.BACKWARD, 100, 0);
//        lol.colorODSBackward(.4,.4);
//        sleep(250);
//        lol.colorODSForward(.2,.2);
//        lol.drivePID(.3,2,Direction.FORWARD,100,0);
//        lol.colorRedMove();
//        lol.blueServo.setPosition(1);
////        lol.redServo.setPosition(.03);
//        lol.turnPID(.3,10,Direction.LEFT, 1.5, 100);
//        lol.drivePID(.6,48, Direction.BACKWARD,100,20);
//        sleep(3000);
//    }
//
//}
//
