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
            robot.imu.getHeading();
            robot.imu.reset();
            telemetry.addData("imu", robot.imu.getNormalized());
            telemetry.update();
//            robot.redServo.setDirection(Servo.Direction.REVERSE);
            robot.redServo.setPower(0);
            robot.armServo.setPosition(0);

        }
        waitForStart();
        double parallel = robot.imu.getNormalized();
        robot.driveTrain.resetEncoders();
        robot.driveByTime(.4, .85);
        robot.shootBall(1, 1.5);
        robot.armMove();
        robot.shootBall(1, 1.5);
        robot.turnUpdate(0.4,30);
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runUsingEncoder();
        robot.touchSensorMoveFront(.4);
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runUsingEncoder();
        robot.driveByTime(-.4, .2);
        robot.driveTrain.runUsingEncoder();
        robot.turnUpdate(.30,robot.turnDiff(parallel,0));
        robot.detectColorRed(0.18);
        robot.pressBeacon();
        robot.driveByTime(-0.3, 1.2);
        robot.turnUpdate(.30,robot.turnDiff(parallel,0));
        robot.detectColorRed(-.18);
        robot.driveByTime(.2, .3);
        robot.pressBeacon();
    }

}

