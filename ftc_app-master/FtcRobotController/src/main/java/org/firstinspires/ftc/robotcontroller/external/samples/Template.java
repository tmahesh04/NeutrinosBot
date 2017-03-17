package org.firstinspires.ftc.robotcontroller.external.samples;

import android.bluetooth.BluetoothA2dp;
        import android.graphics.Color;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "Template", group = "Test") // change name
@Disabled
public class Template extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;
        NeutrinosBEBE lol = new NeutrinosBEBE(telemetry);
        while (!isStarted()) {
            lol.addTelemetry("hello aneesha");
            telemetry.update();
            idle();
        }
        waitForStart();

    }
}