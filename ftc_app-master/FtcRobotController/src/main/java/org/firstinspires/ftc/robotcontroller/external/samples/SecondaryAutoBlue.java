//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import android.bluetooth.BluetoothA2dp;
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
///**
// * Created by Archish on 10/6/16.
// */
//
//@Autonomous(name = "AutoSecondBlue", group = "Test") // change name
//public class SecondaryAutoBlue extends LinearOpMode { // change file name
//    public void main() throws InterruptedException {
//
//    }
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        boolean telemetrizeModules;
//        double LOW_POWER = 0.50;
//        double POWER = 0.70;
//        double HIGH_POWER = 0.90;
//        NeutrinosBEBE lol = new NeutrinosBEBE(telemetry);
////        lol.redServo.setDirection(Servo.Direction.REVERSE);
//        lol.blueServo.setDirection(Servo.Direction.FORWARD);
//        lol.blueServo.setPosition(1);
////        lol.redServo.setPosition(.03);
//        lol.armServo.setPosition(0);
//
//
//        while (!isStarted()) {
//            lol.addTelemetry("hello aneesha");
//            telemetry.update();
//            idle();
//        }
//        lol.drivePID(0.3, 75, Direction.FORWARD, 100, 0);
//        lol.shootBall(1,1.5);
//        lol.armMove();
//        lol.shootBall(1,1.5);
//        lol.drivePID(.3,5,Direction.BACKWARD,100,0);
//        sleep(4000);
//        lol.turnPIDSuperSlow(.6, 65, Direction.LEFT, 2.0, 1000);
//        lol.drivePID(.3, 50, Direction.FORWARD, 100, 0);
//        sleep(3000);
//        lol.drivePID(.5, 25, Direction.BACKWARD, 100 , 0);
//        lol.turnPIDNotAbsolute(.6, 55, Direction.RIGHT, 3.0, 1000);
//        sleep(5000);
//        lol.drivePID(.5, 35, Direction.FORWARD, 100, 0);
//
//
//    }
//}
