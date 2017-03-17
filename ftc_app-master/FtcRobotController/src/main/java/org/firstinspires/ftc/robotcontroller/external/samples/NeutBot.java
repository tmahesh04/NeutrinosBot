package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by NeutrinosFTC on 2/11/2017.
 */

public class NeutBot implements PID_Constants{
   NeutRobot robot = new NeutRobot();
    Motor leftMotor   = new Motor("motor_left");
    Motor rightMotor  = new Motor("motor_right");
    Motor sweepMotor  = new Motor("motor_sweep");
    Motor shootMotor  = new Motor("motor_shoot");
    Servo armServo = FtcOpModeRegister.opModeManager.getHardwareMap().servo.get("ball_stop");
    Servo blueServo = FtcOpModeRegister.opModeManager.getHardwareMap().servo.get("servo_blue");
    Servo redServo = FtcOpModeRegister.opModeManager.getHardwareMap().servo.get("servo_red");
    TouchSensor touchSensorBack = FtcOpModeRegister.opModeManager.getHardwareMap().touchSensor.get("touch_back");
    TouchSensor touchSensorFront = FtcOpModeRegister.opModeManager.getHardwareMap().touchSensor.get("touch_front");
    OpticalDistanceSensor opticalDistanceSensor = FtcOpModeRegister.opModeManager.getHardwareMap().opticalDistanceSensor.get("ODS");
    ColorSensor colorSensor = FtcOpModeRegister.opModeManager.getHardwareMap().colorSensor.get("sensor_color");
    AdafruitIMU imu = new AdafruitIMU("imu");
    private ElapsedTime runtime = new ElapsedTime();
    public NeutBot(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static NeutBot getTelemetry(){
        return instance;
    }

    private static NeutBot instance;

    private Telemetry telemetry;

    public void addTelemetry(String string) {
        telemetry.addLine(string);
    }

    public void addTelemetry(String string, Object data) {
        telemetry.addData(string, data);
    }

    public void addSticky(String string){
        telemetry.log().add(string);
        telemetry.update();
    }

    public void addSticky(String string, Object data){
        telemetry.log().add(string, data);
        telemetry.update();
    }
    turn make = new turn(telemetry);
    private boolean opModeIsActive() {

        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }

    public void drive(double speed, double leftInches, double rightInches) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int leftTarget;
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        // Ensure that the opmode is still active
        while (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftTarget = (int) leftMotor.getCurrentPos();
            newLeftTarget =  ((int) (leftMotor.getCurrentPos() +  (leftInches * COUNTS_PER_INCH)));
            newRightTarget = -1 *  ((int)(rightMotor.getCurrentPos() +  (rightInches * COUNTS_PER_INCH)));
            leftMotor.setDistance(newLeftTarget);
            rightMotor.setDistance(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.runToPosition();
            rightMotor.runToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy())) {
                telemetry.addData("Start Position", leftTarget);
                telemetry.addData("Target Posiition", "Running to %7d ", newLeftTarget);
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.runUsingEncoder();
            rightMotor.runUsingEncoder();
            // optional pause after each move
        }
    }

    public void turnPID(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime) {
        double targetAngle = imu.adjustAngle((DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = power;
        double previousTime = 0;
        Clock clock = new Clock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, Clock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError  * ID;
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * KI_TURN * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            rightMotor.setPower(newPower);
            leftMotor.setPower(newPower);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
    public void turnPIDSlow(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime) {
        double targetAngle = imu.adjustAngle((DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = power;
        double previousTime = 0;
        Clock clock = new Clock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, Clock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError  * ID;
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * KI_TURN * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            rightMotor.setPower(newPower*.5);
            leftMotor.setPower(newPower*.5);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
    public void turnPIDNotAbsolute(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime) {
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = power;
        double previousTime = 0;
        Clock clock = new Clock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, Clock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError  * ID;
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * KI_TURN * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            rightMotor.setPower(newPower);
            leftMotor.setPower(newPower);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public void turnPID(double power, int angle, Direction DIRECTION, double timeout)  {
        turnPID(power, angle, DIRECTION, timeout, 1000);
    }
    public void shootBall(double shootSpeed, double seconds) throws InterruptedException {
        if(opModeIsActive()) {
            shootMotor.setPower(shootSpeed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < seconds)) {}
            shootMotor.setPower(0);
        }
    }
    public void waitNow(long waitTime) {
        try {
            Thread.sleep(waitTime);                 //1000 milliseconds is one second.
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();

        }
    }
    public void armMove ()
            throws InterruptedException {
        armServo.setPosition(.3);
        waitNow(400);
        armServo.setPosition(0);
    }
    public void touchSensorMoveBack(double speed) {
        while (!touchSensorBack.isPressed() && opModeIsActive()) {
            leftMotor.setPower(speed);
            rightMotor.setPower(-speed);
        }
    }
    public void touchSensorMoveFront(double speed) {
        while (!touchSensorFront.isPressed() && opModeIsActive()) {
            leftMotor.setPower(speed);
            rightMotor.setPower(-speed);

        }
    }
    public void colorBlueMove() {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        boolean bLedOn = true;
        boolean bLedOff = false;
        boolean end = false;
        blueServo.setPosition(.57);
        redServo.setPosition(.73);
        colorSensor.enableLed(false);
        while (opModeIsActive() && end == false) {

            colorSensor.enableLed(false);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            if (hsvValues[0] < 10) {
                telemetry.addData("Hue", "red");
                telemetry.update();
                redServo.setPosition(.73);
                blueServo.setPosition(1);
                waitNow(1000);
                end = true;
            } else if (hsvValues[0] > 200 && hsvValues[0] < 260) {
                telemetry.addData("Hue", "blue");
                telemetry.update();
                redServo.setPosition(0);
                blueServo.setPosition(.57);
                waitNow(1000);
                end = true;
            }
        }
    }
    public void colorODSBackward(double leftSpeed, double rightSpeed) {
        boolean end = false;
        while (opModeIsActive() && end == false) {
            double reflectance = opticalDistanceSensor.getRawLightDetected();
            if (reflectance < 1) {
                rightMotor.setPower(rightSpeed);
                leftMotor.setPower(-leftSpeed);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                end = true;
            }
        }
    }

    public void colorRedMove() {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        boolean bLedOn = true;
        boolean bLedOff = false;
        boolean end = false;
        blueServo.setPosition(.3);
        redServo.setPosition(1);
        colorSensor.enableLed(false);
        while (opModeIsActive() && end == false) {

            colorSensor.enableLed(false);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            if (hsvValues[0] < 10) {
                telemetry.addData("Hue", "red");
                telemetry.update();
                redServo.setPosition(1);
                blueServo.setPosition(1);
                waitNow(2000);
                end = true;
            } else if (hsvValues[0] > 200 && hsvValues[0] < 260) {
                telemetry.addData("Hue", "blue");
                telemetry.update();
                redServo.setPosition(0);
                blueServo.setPosition(.3);
                waitNow(2000);
                end = true;
            }
        }
    }


}
