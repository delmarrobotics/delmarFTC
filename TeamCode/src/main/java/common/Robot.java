package common;

/*
 * This file defines a Java Class that performs all the setup and configuration for the robot's hardware (motors and sensors).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {

    // Calculate the COUNTS_PER_INCH for the drive train.
    static final double COUNTS_PER_MOTOR_REV = 28;              // HD Hex Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20;              // Gearing
    static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // 96 mm while converted to inches
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double RAMP_DISTANCE = COUNTS_PER_INCH * 2; // Speed ramp up in encoder counts

    public final double MIN_SPEED        = 0.25;
    public final double MAX_SPEED        = 0.70;
    private static final double MAX_ROTATE_SPEED = 0.50;

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 1.75F;
    public enum COLOR { RED, BLUE}

    // drone launcher servo position
    static final double DRONE_ANGLE_DOWN = 0.48;
    static final double DRONE_ANGLE_UP   = 0.28;
    static final double DRONE_FIRE_DOWN  = 0.063;
    static final double DRONE_FIRE_UP    = 0.16;

    // pixel dropper servo positions
    static final double DROPPER_OPEN     = 0.51;
    static final double DROPPER_CLOSE    = 0.67;

    // Intake
    static final double INTAKE_HOME      = 0.32;
    static final double INTAKE_TARGET    = 0.98;
    static final double SPINNER_SPEED    = 0.2;

    public boolean intakeUp         = true;
    public boolean intakeOn         = false;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;   //  Used to control the left front drive wheel
    public DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotor leftBackDrive = null;    //  Used to control the left back drive wheel
    public DcMotor rightBackDrive = null;   //  Used to control the right back drive wheel

    private DcMotor pixelIntake = null;
    private Servo   intakeRotate = null;
    private CRServo spinnerGray = null;
    private CRServo spinnerBlack = null;
    private CRServo spinnerBucket = null;

    private NormalizedColorSensor colorSensor = null;

    private Servo dropper = null;        // Servo to drop the purple pixel
    private Servo droneAngle = null;
    private Servo droneFire = null;

    private IMU imu;

    public HangingArm hangingArm = null;
    public PixelArm pixelArm = null;

    public Vision vision = null;

    List<DcMotor> motors;

    private final ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    private final LinearOpMode opMode;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initialize the Robot
     */
    public void init() {

        hangingArm = new HangingArm(opMode);
        pixelArm = new PixelArm(opMode);
        vision = new Vision(opMode);

        initDriveTrain();

        colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        try {
            imu = opMode.hardwareMap.get(IMU.class, "imu");

            dropper = opMode.hardwareMap.get(Servo.class, Config.PIXEL_DROPPER);

            droneAngle = opMode.hardwareMap.get(Servo.class, Config.DRONE_ANGLE);
            droneFire = opMode.hardwareMap.get(Servo.class, Config.DRONE_FIRE);

            pixelIntake = opMode.hardwareMap.get(DcMotor.class, Config.PIXEL_INTAKE);
            intakeRotate = opMode.hardwareMap.get(Servo.class, Config.INTAKE_ROTATE);
            spinnerGray = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_GRAY);
            spinnerBlack = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_BLACK);
            spinnerBucket = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_BUCKET);


        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }
    }

    /**
     * Initialize the drive train motors.
     */
    private void initDriveTrain() {
        try {
            leftFrontDrive = opMode.hardwareMap.get(DcMotor.class, Config.LEFT_FRONT);
            rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, Config.RIGHT_FRONT);
            leftBackDrive = opMode.hardwareMap.get(DcMotor.class, Config.LEFT_BACK);
            rightBackDrive = opMode.hardwareMap.get(DcMotor.class, Config.RIGHT_BACK);
        } catch (Exception e) {
            Logger.error(e, "Hardware not found");
        }

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
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

    public void moveRobot(double x, double y, double yaw, double speed) {

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;


        if (x == 0 && y == 0 && yaw == 0 ) {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;

        } else {
            leftFrontPower = x - y - yaw;
            rightFrontPower = x + y + yaw;
            leftBackPower = x + y - yaw;
            rightBackPower = x - y + yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (speed == 0)
                speed = MIN_SPEED;
            else if (speed > MAX_SPEED) {
                speed = MAX_SPEED;
            }
            if (x == 0 && y == 0 && (yaw > 0 || yaw < 0)) {
                if (speed > MAX_ROTATE_SPEED)
                    speed = MAX_ROTATE_SPEED;
            }

            double scale = (1 / max) * speed;

            leftFrontPower *= scale;
            rightFrontPower *= scale;
            leftBackPower *= scale;
            rightBackPower *= scale;

            //Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    /**
     * Stop all the drive train motors.
     */
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    public void resetEncoders() {

        for (DcMotor motor : motors) {
            DcMotor.RunMode mode = leftFrontDrive.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(mode);
        }
    }

    public List<Double> getWheelPositions() {

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add((double)position * COUNTS_PER_INCH);
        }
        return wheelPositions;
    }

    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param speed motor speed (-1 to 1)
     * @param leftInches  distance to move in inches, positive for forward, negative for backward
     * @param rightInches distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds
     */
    public void moveDistance(double speed, double leftInches, double rightInches, double timeout) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            DcMotor.RunMode mode = leftFrontDrive.getMode();

            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            for (DcMotor motor : motors)
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            int leftStart = leftFrontDrive.getCurrentPosition();
            runtime.reset();
            while (opMode.opModeIsActive()) {

                double rampPower = rampSpeed(leftFrontDrive.getCurrentPosition(), leftStart, newLeftTarget, speed);

                leftFrontDrive.setPower(rampPower);
                rightFrontDrive.setPower(rampPower);
                leftBackDrive.setPower(rampPower);
                rightBackDrive.setPower(rampPower);

                /*
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
                        rightBackDrive.isBusy());
                 */

                if (! leftFrontDrive.isBusy())
                    break;

                if (! rightFrontDrive.isBusy())
                  break;

                if (runtime.milliseconds() >= timeout) {
                    Logger.message("encoderDrive timed out");
                    break;
                }
            }

            // Stop all motion;
            for (DcMotor motor : motors)
                motor.setPower(0);

            // Restore run mode to prior state
            for (DcMotor motor : motors)
                motor.setMode(mode);

            Logger.message("moveDistance: target  %6.2f %6.2f  traveled %6.2f %6.2f",
                    (double)newLeftTarget / COUNTS_PER_INCH,
                    (double)newRightTarget / COUNTS_PER_INCH,
                    (double)leftFrontDrive.getCurrentPosition() / COUNTS_PER_INCH,
                    (double)rightFrontDrive.getCurrentPosition() / COUNTS_PER_INCH);
        }
    }

    private double rampSpeed(double current, double start, double end, double speed) {

        double power1 = ((current - start) / RAMP_DISTANCE) * Math.max(MIN_SPEED, speed - MIN_SPEED) + MIN_SPEED;
        double power2 = ((end - current) / RAMP_DISTANCE) * Math.max(MIN_SPEED, speed - MIN_SPEED)  + MIN_SPEED;
        return Math.min(Math.min(power1, power2), speed);
    }

    /**
     * Move the robot until the specified color is detected.
     *
     * @param color the color to detect
     * @param x positive for forward, negative for backwards
     * @param y positive for strafe left ???, negative for strafe right ???
     * @param speed drive speed
     * @param timeout timeout in milliseconds
     */
    public void moveToColor(COLOR color, double x, double y, double speed, double timeout){

        boolean found = false;
        float[] hsvValues = new float[3];
        ElapsedTime elapsedTime = new ElapsedTime();

        elapsedTime.reset();
        moveRobot(x, y, 0, speed);

        while (! found) {
            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV color space
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];

            if (color == COLOR.BLUE) {
                if (hue >= 190 && hue <= 230 && saturation >= .7) {
                    Logger.message("blue line found");
                    found = true;
                }
            } else if (color == COLOR.RED) {
                if ((hue >= 30 && hue <= 90) && saturation >= .5) {
                    Logger.message("red line found");
                    found = true;
                }
            }
            if (elapsedTime.milliseconds() > timeout){
                Logger.message("no line found");
                break;
            }
        }
        stopRobot();
    }

    /**
     * Turn the intake off
     */
    public void intakeOff() {
        pixelIntake.setPower(0);
        spinnerGray.setPower(0);
        spinnerBlack.setPower(0);
        spinnerBucket.setPower(0);
        intakeOn = false;
    }

    /**
     * Toggle the pixel intake on or off.
     */
    public void toggleIntake()
    {
        if(intakeOn) {
            intakeOff();
        } else {
            pixelIntake.setPower(1);
            spinnerGray.setPower(SPINNER_SPEED);
            spinnerBlack.setPower(SPINNER_SPEED);
            spinnerBucket.setPower(SPINNER_SPEED);
            intakeOn = true;
        }
    }

    public void intakeReverse() {

        pixelIntake.setPower(-1);
        intakeOn = false;
    }

    public void toggleIntakeRotate() {
        if (intakeUp) {
            intakeRotate.setPosition(INTAKE_TARGET);
            intakeUp = false;
        } else {
            intakeRotate.setPosition(INTAKE_HOME);
            intakeUp = true;
        }
    }

    /**
     * Drop the preload purple pixel at the current location.
     */
    public void dropPurplePixel(){
        dropper.setPosition(DROPPER_OPEN);
        opMode.sleep(500);
        dropper.setPosition(DROPPER_CLOSE);
    }

    public void dropYellowPixel() {
        pixelArm.positionArm(PixelArm.ARM_POSITION.LOW);
        opMode.sleep(1000);
        dropPixel();
        opMode.sleep(1500);
        pixelArm.positionArm(PixelArm.ARM_POSITION.HOME);
    }

    public void dropPixel () {
        spinnerBucket.setPower(-SPINNER_SPEED);
    }

    /**
     * Launch the drone
     */
    public void launchDrone(){
        droneAngle.setPosition(DRONE_ANGLE_UP);
        opMode.sleep(1000);
        droneFire.setPosition(DRONE_FIRE_UP);
        opMode.sleep(750);
        droneFire.setPosition(DRONE_FIRE_DOWN);
        droneAngle.setPosition(DRONE_ANGLE_DOWN);
    }

    /**
     * Lock in the hanging arm hook
     */
    public void hangRobotLockIn() {
        hangingArm.wristUp();
        opMode.sleep(2000);
        hangingArm.lockInHook();
        moveRobot(.1, 0, 0);
        opMode.sleep(500);
        stopRobot();
    }

    /**
     *  Return the current orientation of the robot.
     * @return orientation in a range of -180 to 180
     */
    public double getOrientation(){

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        Logger.message("Yaw   (Z) %.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        Logger.message("Pitch (X) %.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        Logger.message("Roll  (Y) %.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));

        return orientation.getYaw(AngleUnit.DEGREES);
    }

} // end of class

