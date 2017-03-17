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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name = "helloWorld", group = "THUSHAR")
public class NeutT extends OpMode implements Constants {

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
    Servo redServo;
    Servo blueServo;
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
    public NeutT() {
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
        blueServo = hardwareMap.servo.get("servo_blue");
        redServo = hardwareMap.servo.get("servo_red");
        redServo.setDirection(Servo.Direction.REVERSE);
        blueServo.setDirection(Servo.Direction.FORWARD);
        blueServo.setPosition(1);
        redServo.setPosition(.03);
    }

    @Override
    public void loop() {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        setLeft(left);
        setRight(right);
        if (gamepad2.b){
            ballStop.setPosition(INDEXER_OPEN);
        }
        else {
            ballStop.setPosition(INDEXER_CLOSED);
        }
        if (gamepad2.dpad_down && !sweepPressed){
            setSweeper(COLLECTOR_POWER);
            sweepPressed = true;

        }
        else if (gamepad2.dpad_up && !sweepPressed){
            setSweeper(COLLECTOR_OUT);
            sweepPressed = true;

        }
        else if (gamepad2.dpad_left && sweepPressed){
            setSweeper(0);
            sweepPressed = false;

        }
        if(gamepad2.left_bumper){
            setFlicker(SHOOTER_POWER);
        }
        else if(gamepad2.right_bumper)
        {}

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
    public void setFlicker (double power){
        motorShoot1.setPower(power);
        motorShoot2.setPower(power);
    }
    public void setSweeper(double power){
        motorSweep1.setPower(power);
        motorSweep2.setPower(power);
    }
}
