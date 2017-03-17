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
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by NeutrinosFTC on 2/11/2017.
 */

public class NeutrinosBEBE  implements PID_Constants{

    public static double DEFAULT_SLEEP_TIME = 1000.0;
    public DriveTrain driveTrain = new DriveTrain("left_front", "left_back", "right_front", "right_back");

    private Motor sweepMotor1  = new Motor("motor_sweep1");
    private Motor sweepMotor2 = new Motor("motor_sweep2");
    private Motor shootMotor1  = new Motor("motor_shoot1");
    private Motor shootMotor2 = new Motor("motor_shoot2");
    Servo armServo = FtcOpModeRegister.opModeManager.getHardwareMap().servo.get("ball_stop");
    Servo blueServo = FtcOpModeRegister.opModeManager.getHardwareMap().servo.get("servo_blue");
    CRServo redServo = FtcOpModeRegister.opModeManager.getHardwareMap().crservo.get("servo_red");
    private TouchSensor touchSensorBack = FtcOpModeRegister.opModeManager.getHardwareMap().touchSensor.get("touch_back");
    private TouchSensor touchSensorFront = FtcOpModeRegister.opModeManager.getHardwareMap().touchSensor.get("touch_front");
    private OpticalDistanceSensor opticalDistanceSensor = FtcOpModeRegister.opModeManager.getHardwareMap().opticalDistanceSensor.get("ODS");
    private ColorSensor colorSensor = FtcOpModeRegister.opModeManager.getHardwareMap().colorSensor.get("color_sensor");
    public AdafruitIMU imu = new AdafruitIMU("imu");
    public ElapsedTime runtime = new ElapsedTime();
    public NeutrinosBEBE(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static NeutrinosBEBE getTelemetry(){
        return instance;
    }

    private static NeutrinosBEBE instance;

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
    //turn make = new turn(telemetry);
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
public void drive (double power, int distance, Direction DIRECTION){
    int newDistance = convert(distance);
    driveTrain.resetEncoders();
    driveTrain.setDistance((int) (- newDistance * DIRECTION.value));
    driveTrain.runToPosition();
    while (driveTrain.isBusy() && opModeIsActive()){
        driveTrain.setPower(1,1);
    }
    driveTrain.StopDriving();
    driveTrain.runUsingEncoder();
}
public void drivePID(double power, int distance, Direction DIRECTION, int sleepTime, double targetAngle) {

    int newDistance = convert(distance);
    driveTrain.runUsingEncoder();
    driveTrain.setDistance((int) (-newDistance * DIRECTION.value));
    driveTrain.resetEncoders();
    driveTrain.runToPosition();
    while (driveTrain.rightIsBusy() && opModeIsActive()) {
        double newPowerLeft = power;
        double imuVal = imu.getHeading();
        double error = (targetAngle - imuVal);
        double errorkp = error * KP_STRAIGHT;
        newPowerLeft = (newPowerLeft + (errorkp) * DIRECTION.value);
        driveTrain.setPowerRight(power);
        driveTrain.setPowerLeft(-newPowerLeft);

    }

    driveTrain.StopDriving();
        addSticky("8");
    driveTrain.runUsingEncoder();

    waitNow(sleepTime);

}

    public void drivePID(double power, int distance, double targetAngle, Direction DIRECTION) {

        drivePID(power, distance, DIRECTION, 1000, targetAngle);

    }


    public void drive(double power, int distance, Direction DIRECTION, int sleepTime) {

        double targetAngle = imu.getHeading();

        int newDistance = convert(distance);

        driveTrain.resetEncoders();

        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));

        driveTrain.runToPosition();

        while (driveTrain.rightIsBusy() && opModeIsActive()) {

            double newPowerLeft = power;

            double imuVal = imu.getHeading();

            double error = targetAngle - imuVal;

            double errorkp = error * KP_STRAIGHT;

            newPowerLeft = (newPowerLeft - (errorkp) * DIRECTION.value);

            driveTrain.setPowerLeft(newPowerLeft);

            driveTrain.setPowerRight(power);
        }

        driveTrain.StopDriving();

        driveTrain.runUsingEncoder();

        waitNow(sleepTime);

    }

    public void drivePID(double power, int distance, Direction DIRECTION) {

        drive(power, distance, DIRECTION, 1000);

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
            driveTrain.setPower(power, power);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
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
            driveTrain.setPowerRight(newPower *.4);
            driveTrain.setPowerLeft(-newPower*.4);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
    }

    public void turnPIDCustomKPKIKD(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime, double customKP, double customKI, double customKD) {
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
            double errorkp = currentError * customKP;
            double integralki = currentError * customKI * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * customKD;
            newPower = (errorkp + integralki + dervitivekd);
            driveTrain.setPowerRight(newPower *.4);
            driveTrain.setPowerLeft(-newPower*.4);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
    }
    public void turnPIDSuperSlow(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime) {
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
            driveTrain.setPowerRight(newPower*.25);
            driveTrain.setPowerLeft(newPower*.25);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
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
            driveTrain.setPowerLeft(newPower*.4);
            driveTrain.setPowerRight(newPower*.4);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        driveTrain.StopDriving();
    }

    public void turnPID(double power, int angle, Direction DIRECTION, double timeout)  {
        turnPID(power, angle, DIRECTION, timeout, 1000);
    }
    public void shootBall(double shootSpeed, double seconds) throws InterruptedException {
        if(opModeIsActive()) {
            shootMotor1.setPower(-shootSpeed);
            shootMotor2.setPower(-shootSpeed);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < seconds)) {}
            shootMotor1.setPower(0);
            shootMotor2.setPower(0);
        }
    }
    public void waitNow(long waitTime) {
        try {
            Thread.sleep(waitTime);                 //1000 milliseconds is one second.
        } catch (InterruptedException ex) {
            ex.printStackTrace();

        }
    }
    public void armMove ()
            throws InterruptedException {
        armServo.setPosition(.3);
        waitNow(400);
        armServo.setPosition(0);
    }
    public void touchSensorMoveBack(double maxSpeed, double minSpeed) {
        while (!touchSensorFront.isPressed() && opModeIsActive()) {
            driveTrain.setPowerLeft(-maxSpeed);
            driveTrain.setPowerRight(-maxSpeed);

        }
    }
    public void touchSensorMoveFront(double speed) {
        while (!touchSensorFront.isPressed() && opModeIsActive()) {
            driveTrain.setPowerLeft(-speed);
            driveTrain.setPowerRight(-speed);
        }
    }
    public void colorBlueMove() {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        boolean bLedOn = true;
        boolean bLedOff = false;
        boolean end = false;

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
            if (hsvValues[0] < 200) {
                telemetry.addData("Hue", "red");
                telemetry.update();
                redServo.setPower(1);
                waitNow(750);
                blueServo.setPosition(0);
                waitNow(1000);
                end = true;
            } else if (hsvValues[0] > 200 && hsvValues[0] < 260) {
                telemetry.addData("Hue", "blue");
                telemetry.update();
                redServo.setPower(0);
                blueServo.setPosition(1);
                waitNow(1000);
                end = true;
            }
        }
    }
    public void colorODSBackward(double leftSpeed, double rightSpeed) {
        boolean end = false;
        while (opModeIsActive() && end == false) {
            double reflectance = opticalDistanceSensor.getRawLightDetected();
            if (reflectance < .45) {
                driveTrain.setPowerRight(rightSpeed);
                driveTrain.setPowerLeft(leftSpeed);
            } else {
                driveTrain.StopDriving();
                end = true;
            }
        }
    }
    public void colorODSForward(double leftSpeed, double rightSpeed) {
        boolean end = false;
        while (opModeIsActive() && !end) {
            double reflectance = opticalDistanceSensor.getRawLightDetected();
            if (reflectance < .45) {
                driveTrain.setPowerLeft(-leftSpeed);
                driveTrain.setPowerRight(-rightSpeed);
            } else {
                driveTrain.StopDriving();
                end = true;
             }
        }
    }
    public void colorRedMove() {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        blueServo.setDirection(Servo.Direction.FORWARD);
        boolean bLedOn = true;
        boolean bLedOff = false;
        boolean end = false;
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
            telemetry.addData("Hue",    hsvValues[0]);
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            if (hsvValues[0] < 200) {
                telemetry.addData("Hue", "red");
                telemetry.update();
                redServo.setPower(.53);
                blueServo.setPosition(1);
                waitNow(2000);
                end = true;
            } else if (hsvValues[0] > 200 && hsvValues[0] < 260) {
                telemetry.addData("Hue", "blue");
                telemetry.update();
                redServo.setPower(.03);
                blueServo.setPosition(0);
                waitNow(2000);
                end = true;
            }
        }
    }


    public static int convert(int TICKS) {

        return (int)(TICKS * 35.1070765836);

    }


}
