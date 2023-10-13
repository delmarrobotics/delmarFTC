package common;

/*
 * This file defines a Java Class that performs all the setup and configuration for the robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    public DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    public DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel


    private final ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28;              // HD Hex Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40;              // Gearing  // ToDo verify gearing
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // 96 mm while converted to inches
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.5;
    static final double RAMP_DISTANCE = WHEEL_DIAMETER_INCHES * 2 * COUNTS_PER_INCH; // Speed ramp up in encoder counts
    static final double MIN_SPEED = .02;
    static final double TURN_SPEED = 0.5;


    public ColorSensor colorSensor = null;
    public IMU imu = null;

    public DcMotor lifter = null;       // Motor to lift the robot off the ground
    public Servo dropper = null;        // Servo to drop the purple pixel

    public HangingArm hangingArm = null;

    public Telemetry telemetry;

    /* Declare OpMode members. */
    private HardwareMap hardwareMap;
    public LinearOpMode opMode;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        hangingArm = new HangingArm(opMode);
        telemetry = opMode.telemetry;


    }

    /**
     * Initialize the Robot
     */
    public void init() {

        initDriveTrain();

        // ToDo Check the the configuration file has the correct color sensor hardware device selected.
 //       colorSensor = hardwareMap.get(ColorSensor.class, Config.COLOR_SENSOR);

        imu = hardwareMap.get(IMU.class, Config.IMU);

        lifter = hardwareMap.get(DcMotor.class, Config.LIFTING_WENCH);
    }

    /**
     * Initialize the drive train motors.
     */
    private void initDriveTrain() {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, Config.LEFT_FRONT);
            rightFrontDrive = hardwareMap.get(DcMotor.class, Config.RIGHT_FRONT);
            leftBackDrive = hardwareMap.get(DcMotor.class, Config.LEFT_BACK);
            rightBackDrive = hardwareMap.get(DcMotor.class, Config.RIGHT_BACK);

        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Move robot according to desired axes motions
     *
     * @param x   Positive is forward
     * @param y   Positive is strafe left
     * @param yaw Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {

        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Move the robot forward or backward.
     *
     * @param leftInches  distance to move in inches, positive for forward, negative for backward
     * @param rightInches distance to move in inches, positive for forward, negative for backward
     */
    public void moveByDistance(double speed, double leftInches, double rightInches, double timeoutS) {

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        opMode.waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(speed, leftInches, rightInches, timeoutS);  // S1: Forward 6 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int leftStart;

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            leftStart = leftFrontDrive.getCurrentPosition();

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //leftFrontDrive.setPower(Math.abs(speed));
            //rightFrontDrive.setPower(Math.abs(speed));
            //leftBackDrive.setPower(Math.abs(speed));
            //rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive()) {

                double rampPower = rampSpeed(leftFrontDrive.getCurrentPosition(), leftStart, newLeftTarget, speed);

                leftFrontDrive.setPower(rampPower);
                rightFrontDrive.setPower(rampPower);
                leftBackDrive.setPower(rampPower);
                rightBackDrive.setPower(rampPower);

                Logger.message("power: %4.2f %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     busy: %b  %b  %b  %b",
                        rampPower,
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.isBusy(),
                        rightFrontDrive.isBusy(),
                        leftBackDrive.isBusy(),
                        rightBackDrive.isBusy()
                );


                if (!leftFrontDrive.isBusy())
                    break;

                //             if (!  rightFrontDrive.isBusy())
//                    break

                if (runtime.seconds() >= timeoutS) {
                    Logger.message("encoderDrive timed out");
                    break;
                }

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private double rampSpeed(double current, double start, double end, double speed) {

        double power1 = ((current - start) / RAMP_DISTANCE) * (speed - MIN_SPEED) + MIN_SPEED;
        double power2 = ((end - current) / RAMP_DISTANCE) * (speed - MIN_SPEED) + MIN_SPEED;

        double power = Math.min(Math.min(power1, power2), speed);

        //Logger.message("power %f %f %f", power1, power2, power);

        return power;
    }


    /**
     * Move the robot until the specified color is detected.
     *
     * @param hue the color to detect
     */
    public void moveToColor(double hue){

    }

    /**
     * Rotate the robot to the specified orientation. The robot is at orientation 0 after initialization.
     * Orientation range is -180 (counter clockwise) to 180 (clockwise).
     *
     * @param orientation position -180 to 180
     */
    public void setOrientation(double orientation){
        // ToDo Use SensorIMUOrthogonal.java as an example to create this method
    }

    /**
     *  Return the current orientation of the robot.
     * @return orientation in a range of -180 to 180
     */
    public double getOrientation(){
        double orientation = 0;

        return orientation;
    }

    /**
     * Drop the preload purple pixel at the current location.
     */
    public void dropPixel(){

    }

    /**
     * Turn on the motor that drives the lifting wench
     */
    public void lifterUp(){

        lifter.setPower(1);
    }

    /**
     * Turn on the motor that drives the lifting wench
     */
    public void lifterDown(){
        lifter.setPower(-1);
    }

    /**
     * Turn off the motor that drives the lifting wench
     */
    public void lifterStop(){
        lifter.setPower(0);
    }
}
