package org.firstinspires.ftc.robotcontroller.external.samples;



import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**

 * Created by Archish on 10/28/16.

 */

public class DriveTrain implements PID_Constants {

    private Motor motor1 , motor2, motor3, motor4;

    public DriveTrain(String name1, String name2, String name3, String name4) {

        motor1 = new Motor(name1, DcMotor.Direction.REVERSE);

        motor2 = new Motor(name2, DcMotor.Direction.REVERSE);

        motor3 = new Motor(name3, DcMotor.Direction.FORWARD);

        motor4 = new Motor(name4, DcMotor.Direction.FORWARD);

    }

    public void resetEncoders () {

        motor1.resetEncoder();

        motor2.resetEncoder();

        motor3.resetEncoder();

        motor4.resetEncoder();

    }

    public void setPower (double leftPower, double rightPower) {

        motor1.setPower(leftPower);

        motor2.setPower(leftPower);

        motor3.setPower(rightPower);

        motor4.setPower(rightPower);

    }

    public void setPowerLeft (double power) {

        motor1.setPower(power);

        motor2.setPower(power);

    }

    public void setPowerRight (double power) {

        motor3.setPower(power);

        motor4.setPower(power);

    }

    public void setDistance(int distance){

        motor1.setDistance(distance);

        motor2.setDistance(distance);

        motor3.setDistance(distance);

        motor4.setDistance(distance);

    }

    public void runUsingEncoder() {

        motor1.runUsingEncoder();

        motor2.runUsingEncoder();

        motor3.runUsingEncoder();

        motor4.runUsingEncoder();

    }

    public void runToPosition(){

        motor1.runToPosition();

        motor2.runToPosition();

        motor3.runToPosition();

        motor4.runToPosition();

    }

    public void runWithoutEncoders() {

        motor1.runWithoutEncoders();

        motor2.runWithoutEncoders();

        motor3.runWithoutEncoders();

        motor4.runWithoutEncoders();

    }

    public void StopDriving() {

        setPower(0,0);

    }

    public void setBrakeMode () {

        motor1.setbrakeMode();

        motor2.setbrakeMode();

        motor3.setbrakeMode();

        motor4.setbrakeMode();

    }

    public static int convert(int TICKS) {

        return (int) ((TICKS * 35.1070765836));

    }

    boolean isStalled () {

        int i;

        boolean isStalled;

        if (motor1.isStalled()) i = 1;

        else if (motor2.isStalled()) i = 2;

        else if (motor3.isStalled()) i = 3;

        else if (motor4.isStalled()) i = 4;

        else i = 0;

        isStalled = i >= 3;

        return isStalled;

    }

    public boolean isBusy() {

        return (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy());

    }

    public boolean rightIsBusy(){

        return (motor3.isBusy() && motor4.isBusy());

    }

    public double getCurrentPos () {

        double motor1Pos = motor1.getCurrentPos();

        double motor2Pos = motor2.getCurrentPos();

        double motor3Pos = motor3.getCurrentPos();

        double motor4Pos = motor4.getCurrentPos();

        double motor12Pos = (motor1Pos + motor2Pos) / 2;

        double motor34Pos = (motor3Pos + motor4Pos) / 2;

        double currentPos = (motor12Pos + motor34Pos) / 2;

        return currentPos;
    }
    public void setTwoUsingEncoder()
        {
            motor1.runUsingEncoder();
            motor4.runUsingEncoder();
        }
     public void setTwoDistance(int distance)
     {
         motor1.setDistance(distance);
         motor4.setDistance(distance);
     }
    public void twoRunToPosition()
    {
        motor1.runToPosition();
        motor4.runToPosition();
    }
    public boolean twoIsBusy()
    {
        return (motor1.isBusy() && motor4.isBusy());
    }
    public void setTwoPower(double power)
    {
        motor1.setPower(power);
        motor4.setPower(power);
    }

    }
