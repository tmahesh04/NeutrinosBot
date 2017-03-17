package org.firstinspires.ftc.robotcontroller.external.samples;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
/**
 * Created by NeutrinosFTC on 2/11/2017.
 */

public class turn implements PID_Constants {
    Motor right1 = new Motor("right_front");
    Motor right2 = new Motor("right_back");
    Motor left1 = new Motor("left_front");
    Motor left2 = new Motor("left_back");
    AdafruitIMU imu = new AdafruitIMU("imu");
    public turn(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static turn getTelemetry(){
        return instance;
    }

    private static turn instance;

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
        private boolean opModeIsActive() {

        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void turnPID(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime) {
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
            right1.setPower(newPower);
            right2.setPower(newPower);
            left1.setPower(newPower);
            left2.setPower(newPower);
            prevError = currentError;
            telemetry.addData("TargetAngle", targetAngle);
            telemetry.addData("Heading", imuVAL);
            telemetry.addData("AngleLeftToCover", currentError);
            telemetry.update();
        }
        right1.setPower(0);
        right2.setPower(0);
        left1.setPower(0);
        left2.setPower(0);
    }

    public void turnPID(double power, int angle, Direction DIRECTION, double timeout)  {
        turnPID(power, angle, DIRECTION, timeout, 1000);
    }

}
