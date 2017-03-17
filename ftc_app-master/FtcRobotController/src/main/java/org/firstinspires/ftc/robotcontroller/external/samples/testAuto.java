package org.firstinspires.ftc.robotcontroller.external.samples;

import android.bluetooth.BluetoothA2dp;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Archish on 10/6/16.
 */
@Disabled
@Autonomous(name = "THUSHAR_IS_A_MEME", group = "Test") // change name

public class testAuto extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        turn chimera = new turn(telemetry);
        while (!isStarted()) {
            chimera.addTelemetry("heading", chimera.imu.getHeading());
            telemetry.update();
            idle();
        }
        waitForStart();
        chimera.turnPID(0.7, 90, Direction.LEFT, 3);
        chimera.turnPID(0.7, 90, Direction.LEFT, 3);
        chimera.turnPID(0.7, 90, Direction.LEFT, 3);
        chimera.turnPID(0.7, 90, Direction.LEFT, 3);

    }
}