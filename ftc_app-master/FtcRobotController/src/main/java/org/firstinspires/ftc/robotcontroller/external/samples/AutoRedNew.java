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
 * Created by Thushar on 3/8/17.
 */

@Autonomous(name = "AutoRedNewV12", group = "Test") // change name
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
            robot.redServo.setPower(0);
            robot.armServo.setPosition(0);

        }
        waitForStart();
        robot.driveTrain.resetEncoders();
            robot.driveByTime(.4, 1);
//        robot.encoderDriveForward(.1, 28, 28, 4);
//        sleep(100);
//        robot.shootBall(1, 1.5);
//        robot.armMove();
//        robot.shootBall(1, 1.5);
//        robot.turnPIDSlow(0.3,38, Direction.LEFT, 2.0, 10);
//        sleep(100);
//        robot.driveTrain.resetEncoders();
//        robot.driveTrain.runUsingEncoder();
//        robot.encoderDriveForward(.5, 50, 50, 2);
//        robot.driveTrain.resetEncoders();
//        robot.touchSensorMoveFront(.2);
//        robot.driveTrain.resetEncoders();
//        robot.encoderDriveBackward(.3, 8, 8, 3);
//        robot.driveTrain.runUsingEncoder();
//        robot.turnPIDSlow(.7, -2, Direction.RIGHT, 2.0, 10);
//        sleep(250);
//        robot.detectColorRed(.2);
//        robot.redServo.setPower(-1);
//        sleep(2000);
//        robot.redServo.setPower(1);
//        sleep(1500);
//        robot.redServo.setPower(0);
//        robot.driveTrain.runUsingEncoder();
//        robot.driveTrain.resetEncoders();
//        robot.encoderDriveBackward(.4, 20, 20, 3);
//        robot.turnPIDSlow(.7, 0, Direction.LEFT, 2.0, 10);
//        robot.detectColorRed(-.2);
//        robot.redServo.setPower(-1);
//        sleep(2000);
//        robot.redServo.setPower(1);
//        sleep(1500);
//        robot.redServo.setPower(0);
//        robot.turnPIDNotAbsolute(.7, 10, Direction.RIGHT, 2.0, 10);
//        robot.driveTrain.runUsingEncoder();
//        robot.driveTrain.resetEncoders();
//        robot.encoderDriveBackward(.6, 30,30, 3);
        ////////////////////////////////////////////////////////////////////////////////////////////
//        robot.encoderDriveForward(.3,2,2,4 );
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

