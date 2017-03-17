//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//
//
//import org.firstinspires.ftc.robotcontroller.external.samples.AdafruitIMU;
//
//
//
///**
//
// * Created by Archish on 10/28/16.
//
// */
//
//
//
//public class TankDrive implements PID_Constants {
//
//
//
//    public TankDrive(Telemetry telemetry){
//
//        this.telemetry  = telemetry;
//
//        instance = this;
//
//    }
//
//    public static TankDrive getTelemetry(){
//
//        return instance;
//
//    }
//
//    private static TankDrive instance;
//
//    private Telemetry telemetry;
//
//    public void addTelemetry(String string) {
//
//        telemetry.addLine(string);
//
//    }
//
//    public void addTelemetry(String string, Object data) {
//
//        telemetry.addData(string, data);
//
//    }
//
//    public void addSticky(String string){
//
//        telemetry.log().add(string);
//
//        telemetry.update();
//
//    }
//
//    public void addSticky(String string, Object data){
//
//        telemetry.log().add(string, data);
//
//        telemetry.update();
//
//    }
//
//    public void contUpdate () {
//
//        new Thread(new Runnable() {
//
//            public void run() {
//
//                while (opModeIsActive()) {
//
//                    telemetry.update();
//
//                }
//
//            }
//
//        }).start();
//
//    }
//
//
//
//     Motor and Motor Systems
//
//
//
//
//
//
//
//    /Clock
//
//    Servos
//
//
//
//    IMU
//
//    public AdafruitIMU imu = new AdafruitIMU("imu");
//    NeutRobot robot = new NeutRobot();
//
//    ColorSensor
//
//
//    RangeSensor
//
//
//
//
//    private static final int DEFAULT_SLEEP_TIME = 1000;
//
//    private static final double DEFAULT_TIMEOUT = 3;
//
//
//
//
//
//    private boolean opModeIsActive() {
//
//        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
//
//    }
//
//    public void drivePID(double power, int distance, Direction DIRECTION, int sleepTime, double targetAngle) {
//
//    int newDistance = convert(distance);
//
//        driveTrain.resetEncoders();
//
//        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
//
//        driveTrain.runToPosition();
//
//        while (driveTrain.rightIsBusy() && opModeIsActive()) {
//
//            double newPowerLeft = power;
//
//            double imuVal = imu.getHeading();
//
//            double error = targetAngle - imuVal;
//
//            double errorkp = error * KP_STRAIGHT;
//
//            newPowerLeft = (newPowerLeft - (errorkp) * DIRECTION.value);
//
//            driveTrain.setPowerRight(power);
//
//            driveTrain.setPowerLeft(newPowerLeft);
//
//            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
//
//            TankDrive.getTelemetry().addTelemetry("DistanceLeft", newDistance + driveTrain.getCurrentPos());
//
//            telemetry.update();
//
//        }
//
//        driveTrain.StopDriving();
//
//        driveTrain.runUsingEncoder();
//
//        sleep(sleepTime);
//
//    }
//
//    public void drivePID(double power, int distance, double targetAngle, Direction DIRECTION) {
//
//        drivePID(power, distance, DIRECTION, DEFAULT_SLEEP_TIME, targetAngle);
//
//    }
//
//    public void drivePID(double power, int distance, Direction DIRECTION, int sleepTime) {
//
//        double targetAngle = imu.getHeading();
//
//        int newDistance = convert(distance);
//
//        robot.leftMotor.resetEncoders();
//
//        driveTrain.setDistance((int)((-newDistance) * DIRECTION.value));
//
//        driveTrain.runToPosition();
//
//        while (driveTrain.rightIsBusy() && opModeIsActive()) {
//
//            double newPowerLeft = power;
//
//            double imuVal = imu.getHeading();
//
//            double error = targetAngle - imuVal;
//
//            double errorkp = error * KP_STRAIGHT;
//
//            newPowerLeft = (newPowerLeft - (errorkp) * DIRECTION.value);
//
//            driveTrain.setPowerRight(power);
//
//            driveTrain.setPowerLeft(newPowerLeft);
//
//            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
//
//            TankDrive.getTelemetry().addTelemetry("DistanceLeft", newDistance + driveTrain.getCurrentPos());
//
//            telemetry.update();
//
//        }
//
//        driveTrain.StopDriving();
//
//        driveTrain.runUsingEncoder();
//
//        sleep(sleepTime);
//
//    }
//
//    public void drivePID(double power, int distance, Direction DIRECTION) {
//
//        drivePID(power, distance, DIRECTION, DEFAULT_SLEEP_TIME);
//
//    }
//
//    public void turnPID(double power, int angle, Direction DIRECTION, double timeOut, int sleepTime) {
//
//        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
//
//        double acceptableError = 0.5;
//
//        double currentError = 1;
//
//        double prevError = 0;
//
//        double integral = 0;
//
//        double newPower = power;
//
//        double previousTime = 0;
//
//        Clock clock = new Clock("clock");
//
//        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, Clock.Resolution.SECONDS)) {
//
//            double tChange = System.nanoTime() - previousTime;
//
//            previousTime = System.nanoTime();
//
//            tChange = tChange / 1e9;
//
//            double imuVAL = imu.getHeading();
//
//            currentError = imu.adjustAngle(targetAngle - imuVAL);
//
//            integral += currentError  * ID;
//
//            double errorkp = currentError * KP_TURN;
//
//            double integralki = currentError * KI_TURN * tChange;
//
//            double dervitive = (currentError - prevError) / tChange;
//
//            double dervitivekd = dervitive * KD_TURN;
//
//            newPower = (errorkp + integralki + dervitivekd);
//
//            robot.rightMotor.setPower(-newPower);
//
//            robot.rightMotor.setPower(newPower);
//
//            prevError = currentError;
//
//            TankDrive.getTelemetry().addTelemetry("TargetAngle", targetAngle);
//
//            TankDrive.getTelemetry().addTelemetry("Heading", imuVAL);
//
//            TankDrive.getTelemetry().addTelemetry("AngleLeftToCover", currentError);
//
//            telemetry.update();
//
//        }
//
//        robot.rightMotor.setPower(0);
//        robot.leftMotor.setPower(0);
//        sleep(sleepTime);
//
//    }
//
//    public void turnPID(double power, int angle, Direction DIRECTION, double timeout)  {
//
//        turnPID(power, angle, DIRECTION, timeout, DEFAULT_SLEEP_TIME);
//
//    }
//
//    public void turnPID(double power, int angle, Direction DIRECTION)  {
//
//        turnPID(power, angle, DIRECTION, DEFAULT_TIMEOUT);
//
//    }
//
//    public void drivePIDRange(double power, int distance, int sleepTime) {
//
//        double targetAngle = imu.getHeading();
//
//        driveTrain.runUsingEncoder();
//
//        while (rangeSensor.rawUltrasonic() > distance && opModeIsActive()) {
//
//            double newPowerLeft = power;
//
//            double imuVal = imu.getHeading();
//
//            double error = targetAngle - imuVal;
//
//            double errorkp = error * KP_STRAIGHT;
//
//            newPowerLeft = (newPowerLeft - (errorkp));
//
//            driveTrain.setPowerRight(-power);
//
//            driveTrain.setPowerLeft(-newPowerLeft);
//
//            TankDrive.getTelemetry().addTelemetry("Heading", imuVal);
//
//            TankDrive.getTelemetry().addTelemetry("Ultrasonic", rangeSensor.rawUltrasonic());
//
//            telemetry.update();
//
//        }
//
//        driveTrain.StopDriving();
//
//        driveTrain.runWithoutEncoders();
//
//        sleep(sleepTime);
//
//    }
//
//    public void drivePIDRange(double power, int distance ) {
//
//        drivePIDRange(power, distance, DEFAULT_SLEEP_TIME);
//
//    }
//
//    public void setBrakeMode(int time) {
//
//        driveTrain.setBrakeMode();
//
//        sleep(time);
//
//    }
//
//    public void stopRed(double power, Direction Direction) {
//
//        driveTrain.runUsingEncoder();
//
//        double targetAngle = imu.getHeading();
//
//        while ((!beaconColor.detectRed()) && opModeIsActive()){
//
//            double newPower = power;
//
//            double heading = imu.getHeading();
//
//            double error = targetAngle - heading;
//
//            double errorkp = error * KP_STRAIGHT;
//
//            newPower = newPower - (errorkp * Direction.value);
//
//            driveTrain.setPowerLeft(power * Direction.value);
//
//            driveTrain.setPowerRight(newPower * Direction.value);
//
//            TankDrive.getTelemetry().addTelemetry("Heading", heading);
//
//            TankDrive.getTelemetry().addTelemetry("red Val", beaconColor.colorNumber());
//
//            telemetry.update();
//
//        }
//
//        driveTrain.StopDriving();
//
//    }
//
//    public void stopBlue(double power, Direction Direction) {
//
//        driveTrain.runUsingEncoder();
//
//        double targetAngle = imu.getHeading();
//
//        while ((!beaconColor.detectBlue()) && opModeIsActive()){
//
//            double newPower = power;
//
//            double heading = imu.getHeading();
//
//            double error = targetAngle - heading;
//
//            double errorkp = error * KP_STRAIGHT;
//
//            newPower = newPower - (errorkp * Direction.value);
//
//            driveTrain.setPowerLeft(power * Direction.value);
//
//            driveTrain.setPowerRight(newPower * Direction.value);
//
//            TankDrive.getTelemetry().addTelemetry("Heading", heading);
//
//            TankDrive.getTelemetry().addTelemetry("Blue Val", beaconColor.colorNumber());
//
//            telemetry.update();
//
//        }
//
//        driveTrain.StopDriving();
//
//    }
//
//    rangeSensor
//
//    setPower
//
//    public void setPowerLeft(double power) {
//
//        driveTrain.setPowerLeft(power);
//
//    }
//
//
//
//    public void setPowerRight(double power) {
//
//        driveTrain.setPowerRight(power);
//
//    }
//
//
//
//    public void setPowerCollector(double powerCollector) {
//
//        collector.setPower(powerCollector);
//
//    }
//
//
//
//    public void setPowerShooter(double powerShooter){
//
//        shooter.runWithoutEncoders();
//
//        shooter.setPower(powerShooter);
//
//    }
//
//
//
//    public void sleep() {
//
//        sleep(1000);
//
//    }
//
//
//
//    public void sleep(int time) {
//
//        try {
//
//            Thread.sleep((long) time);
//
//        } catch (InterruptedException ex) {
//
//            Thread.currentThread().interrupt();
//
//        }
//
//    }
//
//
//
//    servos
//
//    public void setCapClamp (double position) {
//
//        capClamp.setPosition(position);
//
//    }
//
//    public void setIndexer(double position) {
//
//        indexer.setPosition(position);
//
//    }
//
//
//
//    public void setBeaconPresser(double power) {
//
//        beaconPresser.setPower(power);
//
//    }
//
//
//
//    public void runAllTelemetry() {
//
//        runSensorTelemetry();
//
//        driveTrain.telemetryRun();
//
//        collector.telemetryRun(true);
//
//        shooter.telemetryRun(true);
//
//        beaconPresser.telemetryRun();
//
//        indexer.telemetryRun();
//
//    }
//
//    public void runSensorTelemetry () {
//
//        imu.telemetryRun();
//
//        beaconColor.telemetryRun();
//
//        rangeSensor.telemetryRun();
//
//    }
//
//
//
//
//
//
//
//
//
//}