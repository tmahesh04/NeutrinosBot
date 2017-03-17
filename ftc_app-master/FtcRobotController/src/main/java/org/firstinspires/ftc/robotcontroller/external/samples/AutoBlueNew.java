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

@Autonomous(name = "AutoBlueNew3", group = "Test") // change name
public class AutoBlueNew extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;


        NeutrinosBEBE robot = new NeutrinosBEBE(telemetry);
        while (!isStarted()) {
            telemetry.addLine("hello aneesha");
            telemetry.addData("imu", robot.imu.getHeading());
            telemetry.update();

        }
        waitForStart();
        robot.drive(1, 90, Direction.BACKWARD);
        robot.blueServo.setDirection(Servo.Direction.FORWARD);
        robot.blueServo.setPosition(1);
//        robot.redServo.setPosition(.03);
        robot.armServo.setPosition(0);
        robot.drivePID(1,38,Direction.FORWARD, 100 ,0);
        robot.shootBall(1, 1.5);
        robot.armMove();
        robot.shootBall(1, 1.5);
        robot.turnPIDSlow(0.3, 125, Direction.LEFT, 2.5, 1000);
        robot.drivePID(.7,78, Direction.BACKWARD,100,45);
        sleep(250);
        robot.touchSensorMoveBack(.3, .8);
        robot.drivePID(.4,2,Direction.FORWARD,100, 45);
        robot.turnPIDSuperSlow(.3, 183, Direction.LEFT, 4.5, 1000);
        robot.colorODSBackward(.4,.4);
        sleep(250);
        robot.colorODSForward(.2,.2);
        sleep(250);
        robot.colorBlueMove();
        robot.turnPIDSlow(.3, 183, Direction.LEFT, 1.0,0);
        robot.blueServo.setPosition(1);
//        robot.redServo.setPosition(.03);
        robot.drivePID(.3, 22, Direction.FORWARD, 100, 0);
        robot.colorODSForward(.3,.3);
        robot.colorODSBackward(.2,.2);
        sleep(250);
        robot.colorBlueMove();
        robot.blueServo.setPosition(1);
//        robot.redServo.setPosition(.03);
        sleep(250);
        robot.turnPIDNotAbsolute(.5, 30, Direction.RIGHT,1,0);
        robot.drivePID(.7,40, Direction.FORWARD, 100, 20);
        sleep(3000);
    }

}

