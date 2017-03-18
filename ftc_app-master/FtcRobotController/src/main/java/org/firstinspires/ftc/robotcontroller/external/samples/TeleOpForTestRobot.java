/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TeleOpForTestRobot extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.


        static final double SCALEDOWN = 0.15;


        DcMotor rightMotor1;
        DcMotor rightMotor2;
        DcMotor leftMotor1;
        DcMotor leftMotor2;
        DcMotor motorSweep1;
        DcMotor motorSweep2;
        DcMotor motorShoot1;
        DcMotor motorShoot2;
        Servo ballStop;
        CRServo redServo;
        Servo boxLeft;
        Servo boxRight;
        Servo boxTilt;
        Servo climberSwerve;
        Servo climberDump;
        Servo hookRight;
        Servo hookLeft;
        TouchSensor touch;
        TouchSensor touchLift;

        boolean rightOpenPressed = false;
        boolean leftOpenPressed = false;
        boolean sweepPressed = false;
        boolean shooterPressed = false;
        boolean swervePressed = false;
        boolean switchPressed = false;
        boolean slowPressed = false;
        boolean hookPressed = false;
        double sweepPower = 0;
        double shooterPower = 0;
        int counter = 0;
        double finalPosition;

    /**
     * Constructor
     */
    public TeleOpForTestRobot() {
    }
    @Override
    public void init() {
        rightMotor1 = hardwareMap.dcMotor.get("right_front");
        leftMotor1 = hardwareMap.dcMotor.get("left_front");
        rightMotor2 = hardwareMap.dcMotor.get("right_back");
        leftMotor2 = hardwareMap.dcMotor.get("left_back");
        ballStop = hardwareMap.servo.get("ball_stop");
        motorShoot1 = hardwareMap.dcMotor.get("motor_shoot1");
        motorShoot2 = hardwareMap.dcMotor.get("motor_shoot2");
        motorSweep1 = hardwareMap.dcMotor.get("motor_sweep1");
        motorSweep2 = hardwareMap.dcMotor.get("motor_sweep2");
        redServo = hardwareMap.crservo.get("servo_red");

    }

    @Override
    public void loop() {

        redServo.setPower(0);
        float right = -gamepad1.left_stick_y;
        float left = gamepad1.right_stick_y;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);


        float speedCheck = gamepad1.left_trigger;

        speedCheck = Range.clip(speedCheck, 0, 1);

        if (speedCheck > 0.5) {
            rightMotor1.setPower(right * SCALEDOWN);
            rightMotor2.setPower(right * SCALEDOWN);

            leftMotor1.setPower(left * SCALEDOWN);
            leftMotor2.setPower(left * SCALEDOWN);

        } else {
            setRight(-right);
            setLeft(-left);
        }
        if (gamepad2.b) {
            ballStop.setPosition(.3);

            waitNow(400);
            ballStop.setPosition(0);

        }
        if (gamepad2.dpad_down && !sweepPressed) {
            sweepPower = 0.9;
            sweepPressed = true;
        } else if (gamepad2.dpad_up && !sweepPressed) {
                sweepPower = -0.9;
            sweepPressed = true;
        } else if (gamepad2.dpad_left && sweepPressed) {
            sweepPower = 0;
            sweepPressed = false;
        }
        motorSweep1.setPower(sweepPower);
        motorSweep2.setPower(sweepPower);

        if (gamepad2.right_bumper) {
            shooterPower = .8;
        } else if (gamepad2.left_bumper) {
            shooterPower = 0;
        }
        motorShoot1.setPower(-shooterPower);
        motorShoot2.setPower(-shooterPower);
    }



    public void waitNow(long waitTime) {
        try {
            Thread.sleep(waitTime);                 //1000 milliseconds is one second.
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();

        }
    }
    public void setRight(double power)
    {
        rightMotor1.setPower(power);
        rightMotor2.setPower(power);
    }
    public void setLeft(double power)
    {
        leftMotor1.setPower(power);
        leftMotor2.setPower(power);
    }
    public void setAll(double power)
    {
        rightMotor1.setPower(power);
        rightMotor2.setPower(power);
        leftMotor1.setPower(power);
        leftMotor2.setPower(power);
    }
}
