package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by NeutrinosFTC on 3/12/2017.
 */

public abstract class NeutLinearOpMode extends LinearOpMode {


    public NeutrinosBEBE robot;

    public final void runOpMode() throws InterruptedException {

        try {
            robot = new NeutrinosBEBE(super.telemetry);
            runLinearOpMode();

        } finally {

            stopLinearOpMode();

        }

    }



    public abstract void runLinearOpMode() throws InterruptedException;

    public void stopLinearOpMode() {}

}
