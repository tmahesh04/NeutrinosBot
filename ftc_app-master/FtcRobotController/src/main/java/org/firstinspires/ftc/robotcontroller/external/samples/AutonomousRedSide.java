///*
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Robert Atkinson nor the names of his contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.robotcontroller.external.samples;

//
///**
// * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
// * It uses the common Pushbot hardware class to define the drive on the robot.
// * The code is structured as a LinearOpMode
// *
// * The code REQUIRES that you DO have encoders on the wheels,
// *   otherwise you would use: PushbotAutoDriveByTime;
// *
// *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
// *   otherwise you would use: PushbotAutoDriveByEncoder;
// *
// *  This code requires that the drive Motors have been configured such that a positive
// *  power command moves them forward, and causes the encoders to count UP.
// *
// *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
// *
// *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
// *  This is performed when the INIT button is pressed on the Driver Station.
// *  This code assumes that the robot is stationary when the INIT button is pressed.
// *  If this is not the case, then the INIT should be performed again.
// *
// *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
// *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
// *
// *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
// *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
// *  This is consistent with the FTC field coordinate conventions set out in the document:
// *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@Autonomous(name="Pushbot: Auwqeto Drive By Gyro", group="Pushbot")
//@Disabled
//public class AutonomousRedSide extends LinearOpMode implements PID_Constants {
//
//    /* Declare OpMode members. */
//    NeutRobot         robot   = new NeutRobot();   // Use a Pushbot's hardware
//    private ElapsedTime     runtime = new ElapsedTime();
//    AdafruitIMU imu = new AdafruitIMU("imu");
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
//
//    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    // These constants define the desired driving/control characteristics
//    // The can/should be tweaked to suite the specific robot drive train.
//    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
//    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.
//
//    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
//    static final double     P_TURN_COEFF            = 0.02;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
//    static final double     DEFAULT_SLEEP_TIME      = 1000.0;
//    static final double     DEFAULT_TIMEOUT         = 3;
//    ColorSensor colorSensor;
//    Servo armServo;
//    Servo redServo;
//    Servo blueServo;
//    OpticalDistanceSensor opticalDistanceSensor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        /*
//         * Initialize the standard drive system variables.
//         * The init() method of the hardware class does most of the work here
//         */
//        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
//        armServo = hardwareMap.servo.get("ball_stop");
//        blueServo = hardwareMap.servo.get("servo_blue");
//        redServo = hardwareMap.servo.get("servo_red");
//        colorSensor = hardwareMap.colorSensor.get("sensor_color");
//        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("ODS");
//        redServo.setDirection(Servo.Direction.REVERSE);
//        blueServo.setDirection(Servo.Direction.REVERSE);
//
//
//        blueServo.setPosition(.1);
//        redServo.setPosition(.1);
//        armServo.setPosition(.5);
//        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Send telemetry message to alert driver that we are calibrating;
//        // make sure the gyro is calibrated before continuing
//
//
//        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
//
//        waitForStart();
//
//        encoderDrive(.5, 33, 33);
//        shootBall(.8, 2);
//        armMove();
//        shootBall(.8, 2);
//        turnPID(.2, 45, Direction.LEFT, DEFAULT_TIMEOUT, 100);
//        encoderDrive(.8, 70, 70);
//        touchSensorMoveFront(.3);
//        sleep(100);
//        encoderDrive(.5, -6, -6);
//        turnPID(.2, 0, Direction.RIGHT, DEFAULT_TIMEOUT, 100);
//        colorODSForward(.4, .4);
//        sleep(250);
//        colorODSBackward(.2, .2);
//        sleep(250);
//        encoderDrive(.3, 1, 1);
//        colorRedMove();
//        sleep(250);
//        blueServo.setPosition(.025);
//        redServo.setPosition(.1);
//        sleep(250);
//        encoderDrive(.3, -18, -18);
//        sleep(250);
//        turnPID(.2, 0, Direction.LEFT, 1, 100);
//        colorODSBackward(.3, .3);
//        sleep(250);
//        encoderDrive(.3, 2, 2);
//        colorRedMove();
//        sleep(250);
//        blueServo.setPosition(.025);
//        redServo.setPosition(.1);
//        sleep(250);
//        encoderDrive(.5, 35, 35);
//        gyro.resetZAxisIntegrator();
//        gyroTurn(.4, 60);
//        encoderDrive(.5, -66, -66);
//    }
//
//
//
//
//    public void gyroDrive ( double speed,
//                            double distance,
//                            double angle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int)(distance * COUNTS_PER_INCH);
//            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            robot.leftMotor.setTargetPosition(newLeftTarget);
//            robot.rightMotor.setTargetPosition(newRightTarget);
//
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.leftMotor.setPower(-speed);
//            robot.rightMotor.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
//
//                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if any one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.leftMotor.setPower(leftSpeed);
//                robot.rightMotor.setPower(rightSpeed);
//
//                // Display drive status for the driver.
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
//                        telemetry.addData("Z Value",                 gyro.getIntegratedZValue()));
//                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.leftMotor.setPower(0);
//            robot.rightMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    /**
//     *  Method to spin on central axis to point in a new direction.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the heading (angle)
//     *  2) Driver stops the opmode running.
//     *
//     * @param speed Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     */
//    public void gyroTurn (  double speed, double angle) {
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//    }
//
//    /**
//     *  Method to obtain & hold a heading for a finite amount of time
//     *  Move will stop once the requested time has elapsed
//     *
//     * @param speed      Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     * @param holdTime   Length of time (in seconds) to hold the specified heading.
//     */
//    public void gyroHold( double speed, double angle, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            // Update telemetry & Allow time for other processes to run.
//            onHeading(speed, angle, P_TURN_COEFF);
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        robot.leftMotor.setPower(0);
//        robot.rightMotor.setPower(0);
//    }
//
//    /**
//     * Perform one cycle of closed loop heading control.
//     *
//     * @param speed     Desired speed of turn.
//     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
//     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                  If a relative angle is required, add/subtract from current heading.
//     * @param PCoeff    Proportional Gain coefficient
//     * @return
//     */
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        error = getError(angle);
//
//        if (Math.abs(error) <= 2) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed*steer;
//            leftSpeed   = -1*rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//        robot.leftMotor.setPower(leftSpeed);
//        robot.rightMotor.setPower(rightSpeed);
//
//        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//
//    /**
//     * getError determines the error between the target angle and the robot's current heading
//     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
//     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
//     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
//     */
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - gyro.getIntegratedZValue();
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//    /**
//     * returns desired steering force.  +/- 1 range.  +ve = steer left
//     * @param error   Error angle in robot relative degrees
//     * @param PCoeff  Proportional Gain Coefficient
//     * @return
//     */
//    public double getSteer(double error, double PCoeff) {
//        return Range.clip(error * PCoeff, -1, 1);
//    }
//
//
//
//
//    public void touchSensorMoveFront(double speed) {
//        while (!robot.touchSensorFront.isPressed()) {
//            robot.leftMotor.setPower(speed);
//            robot.rightMotor.setPower(speed);
//
//        }
//    }
//
//    public void touchSensorMoveBack(double speed) {
//        while (!robot.touchSensorBack.isPressed()) {
//            robot.leftMotor.setPower(-speed);
//            robot.rightMotor.setPower(-speed);
//        }
//    }
//
//    public void colorRedMove() {
//        float hsvValues[] = {0F, 0F, 0F};
//        final float values[] = hsvValues;
//        boolean bLedOn = true;
//        boolean bLedOff = false;
//        boolean end = false;
//        blueServo.setPosition(0.25);
//        redServo.setPosition(.1);
//        colorSensor.enableLed(false);
//        while (opModeIsActive() && end == false) {
//
//            colorSensor.enableLed(false);
//
//            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
//
//            // send the info back to driver station using telemetry function.
//            telemetry.addData("LED", bLedOn ? "On" : "Off");
//            telemetry.addData("Clear", colorSensor.alpha());
//            telemetry.addData("Red  ", colorSensor.red());
//            telemetry.addData("Green", colorSensor.green());
//            telemetry.addData("Blue ", colorSensor.blue());
//            int red = colorSensor.red();
//            int blue = colorSensor.blue();
//            if (hsvValues[0] < 10) {
//                telemetry.addData("Hue", "red");
//                telemetry.update();
//                redServo.setPosition(.25);
//                blueServo.setPosition(.5);
//                waitNow(2000);
//                end = true;
//            } else if (hsvValues[0] > 200 && hsvValues[0] < 260) {
//                telemetry.addData("Hue", "blue");
//                telemetry.update();
//                redServo.setPosition(1.05);
//                blueServo.setPosition(.025);
//                waitNow(2000);
//                end = true;
//            }
//        }
//    }
//
//
//    public void colorODSBackward(double leftSpeed, double rightSpeed) {
//        boolean end = false;
//        while (opModeIsActive() && end == false) {
//            double reflectance = opticalDistanceSensor.getRawLightDetected();
//            if (reflectance < 1) {
//                robot.rightMotor.setPower(-rightSpeed);
//                robot.leftMotor.setPower(-leftSpeed);
//            } else {
//                robot.leftMotor.setPower(0);
//                robot.rightMotor.setPower(0);
//                end = true;
//            }
//        }
//    }
//
//    public void colorODSForward(double leftSpeed, double rightSpeed) {
//        boolean end = false;
//        while (opModeIsActive() && end == false) {
//            double reflectance = opticalDistanceSensor.getRawLightDetected();
//            if (reflectance < 1) {
//                robot.rightMotor.setPower(rightSpeed);
//                robot.leftMotor.setPower(leftSpeed);
//            } else {
//                robot.leftMotor.setPower(0);
//                robot.rightMotor.setPower(0);
//                end = true;
//            }
//        }
//    }
//
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches
//    ) throws InterruptedException {
//        int newLeftTarget;
//        int newRightTarget;
//        int leftTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            leftTarget = robot.leftMotor.getCurrentPosition();
//            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
//            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
//            robot.leftMotor.setTargetPosition(newLeftTarget);
//            robot.rightMotor.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.leftMotor.setPower(Math.abs(speed));
//            robot.rightMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            while (opModeIsActive() &&
//                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {
//
//                // Display it for the driver.
//                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                //telemetry.addData("Path2",  "Running at %7d :%7d",
//                //robot.leftMotor.getCurrentPosition(),
//                //robot.rightMotor.getCurrentPosition());
//                telemetry.addData("Start Position", leftTarget);
//                telemetry.addData("Target Posiition", "Running to %7d ", newLeftTarget);
//                telemetry.update();
//                //telemetry.update();
//
//                // Allow time for other processes to run.
//                idle();
//            }
//
//            // Stop all motion;
//            robot.leftMotor.setPower(0);
//            robot.rightMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            // optional pause after each move
//        }
//    }
//    public void shootBall(double shootSpeed,
//                          double seconds) throws InterruptedException {
//        if(opModeIsActive()) {
//
//            robot.shootMotor.setPower(shootSpeed);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < seconds)) {
//
//            }
//
//            robot.shootMotor.setPower(0);
//        }
//    }
//    public void armMove ()
//            throws InterruptedException {
//        armServo.setPosition(.8);
//        waitNow(400);
//        armServo.setPosition(.5);
//    }
//    public void waitNow(long waitTime)
//    {
//        try {
//            Thread.sleep(waitTime);                 //1000 milliseconds is one second.
//        } catch (InterruptedException ex) {
//            Thread.currentThread().interrupt();
//
//        }
//
//
//    }
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
//            telemetry.addData("TargetAngle", targetAngle);
//
//            telemetry.addData("Heading", imuVAL);
//
//            telemetry.addData("AngleLeftToCover", currentError);
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
//        turnPID(power, angle, DIRECTION, timeout, 1000);
//
//    }
//
//
//
//}
//
