/*
 * This this contains the class that controls the robot's drive train
 */

package common;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public class Drive extends Thread {

    static final boolean LOG_VERBOSE = false;

    public static double DRIVE_FACTOR  = 1;
    public static double STRAFE_FACTOR = 1.11;
    public static double TURN_FACTOR = (24.9/2);



    // Drive train
    private final double COUNTS_PER_MOTOR_REV = 28;              // HD Hex Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 20;              // Gearing
    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // 96 mm wheels converted to inches
    private final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final double RAMP_DISTANCE = COUNTS_PER_INCH * 12;   // ramp down distance in encoder counts
    private final double RAMP_TIME = 1000;                  // ramp up time in milliseconds
    private final double RAMP_MIN_SPEED = 0.2;

    private final double MIN_SPEED = 0.25;
    private final double MAX_SPEED = 0.9;
    private final double MAX_ROTATE_SPEED = 0.50;

    public enum DIRECTION { FORWARD, BACK, LEFT, RIGHT, TURN_LEFT, TURN_RIGHT, DRIVER, STOOPED }

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR {RED, BLUE}

    //  Drive train motors
    public DcMotorEx leftFrontDrive = null;   //  Used to control the left front drive wheel
    public DcMotorEx rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotorEx leftBackDrive = null;    //  Used to control the left back drive wheel
    public DcMotorEx rightBackDrive = null;   //  Used to control the right back drive wheel

    public  Encoder sideEncoder;

    private IMU imu;
    private NormalizedColorSensor colorSensor = null;

    public DistanceSensor distanceSensor;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    private boolean running = true;
    private boolean moving = false;

    private int leftFrontStartPos  = 0;
    private int rightFrontStartPos = 0;
    private int leftBackStartPos   = 0;
    private int rightBackStartPos  = 0;

    private DIRECTION lastDirection = DIRECTION.STOOPED;

    List<DcMotorEx> motors;
    LinearOpMode opMode;

    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
        this.setName("Drive");
        init();
    }

    /**
     * Initialize the drive train motors.
     */
    private void init() {
        try {
            leftFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
            rightFrontDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
            leftBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
            rightBackDrive = opMode.hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

            imu = opMode.hardwareMap.get(IMU.class, "imu");

            colorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, Config.COLOR_SENSOR);
            colorSensor.setGain(COLOR_SENSOR_GAIN);

            distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, Config.DISTANCE_SENSOR);

            sideEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, Config.INTAKE_ROTATE));

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
     * Drive the robot with gamepad 1 joysticks one a separate thread
     */
    public void run() {

        while (!opMode.isStarted()) Thread.yield();
       Logger.message("robot drive thread started");

        ElapsedTime driveTime = new ElapsedTime();
        double lastTime = driveTime.milliseconds();
        double lastSpeed = 0;
        double accelerationPerMS = (MAX_SPEED - MIN_SPEED) / (1000 * 1.5);   // 1.5 second to accelerate to full speed

        while (running && opMode.opModeIsActive()) {

            // Left stick to go forward back and strafe. Right stick to rotate. Left trigger accelerate.
            Gamepad gamepad = opMode.gamepad1;
            double x = -gamepad.left_stick_y / 2.0;  // Reduce drive rate to 50%.
            double y = -gamepad.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
            double yaw = -gamepad.right_stick_x / 3.0;  // Reduce rotate rate to 33%.
            double speed = (gamepad.left_trigger * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;

            // limit acceleration to prevent skidding.
            double currentTime = driveTime.milliseconds();
            if (! moving) {
                speed = MIN_SPEED;
            } else if (speed > lastSpeed) {
                double deltaTime = currentTime - lastTime;
                double acceleration = (speed - lastSpeed) / (deltaTime);
                if (acceleration > (accelerationPerMS * deltaTime))
                    speed = lastSpeed + (accelerationPerMS * deltaTime);
            }
            lastTime = currentTime;
            lastSpeed = speed;

            if (speed > MAX_SPEED) speed = MAX_SPEED;
            if (x == 0 && y == 0 && yaw != 0) {
                if (speed > MAX_ROTATE_SPEED) speed = MAX_ROTATE_SPEED;
            }

            if (x != 0 || y != 0 || yaw != 0) {
                DIRECTION direction;
                double MAX_STICK = 0.5;

                if (yaw != 0) {
                    direction =  DIRECTION.DRIVER;
                } else if (Math.abs(x) == MAX_STICK || (x != 0 && y == 0 )) {
                    if (x > 0)
                        direction = DIRECTION.FORWARD;
                    else
                        direction = DIRECTION.BACK;
                } else if (Math.abs(y) == MAX_STICK || (x == 0 /*&& y != 0 */ )) {
                    if (y > 0)
                        direction =  DIRECTION.LEFT;
                    else
                        direction =  DIRECTION.RIGHT;
                } else {
                    direction =  DIRECTION.DRIVER;
                }

                if (direction == DIRECTION.DRIVER) {
                    moveRobot(x, y, yaw, speed);
                } else {
                    moveRobot(direction, speed);
                }

                moving = true;
                lastDirection = direction;
                Logger.message("%-12s   %6.2f %6.2f %6.2f  %6.2f   %6.2f ", direction, x , y, yaw, gamepad.left_trigger, speed);

            } else if (moving) {
                stopRobot();
                lastDirection = DIRECTION.STOOPED;
                moving = false;

            } else {
                Thread.yield();
            }
        }
        Logger.message("robot drive thread stopped");
    }

    /**
     * Stop the thread's run method
     */
    public void end () {
        running = false;
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

            // ToDo remove, done in run
            if (speed == 0)
                speed = MIN_SPEED;
            else if (speed > MAX_SPEED) {
                speed = MAX_SPEED;
            }
            if (x == 0 && y == 0 && yaw != 0) {
                if (speed > MAX_ROTATE_SPEED)
                    speed = MAX_ROTATE_SPEED;
            }

            double scale = (1 / max) * speed;

            leftFrontPower *= scale;
            rightFrontPower *= scale;
            leftBackPower *= scale;
            rightBackPower *= scale;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    /**
     *  Method to move the robot in the specified direction. The encoder are used the correct for drift
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     */
    public void moveRobot(DIRECTION direction, double speed) {

        if (!opMode.opModeIsActive()) return;

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        // If the direction have changed get the encoder positions.
        if (direction != lastDirection) {
            leftFrontStartPos = leftFrontDrive.getCurrentPosition();
            rightFrontStartPos = rightFrontDrive.getCurrentPosition();
            leftBackStartPos = leftBackDrive.getCurrentPosition();
            rightBackStartPos = rightBackDrive.getCurrentPosition();
        }

        // Correct for drift
        double leftFrontPos =  Math.abs(leftFrontDrive.getCurrentPosition()  - leftFrontStartPos);
        double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition() - rightFrontStartPos);
        double leftBackPos =   Math.abs(leftBackDrive.getCurrentPosition()   - leftBackStartPos);
        double rightBackPos =  Math.abs(rightBackDrive.getCurrentPosition()  - rightBackStartPos);
        double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

        double scale = .0015;
        double leftFrontAdjust = (maxPos - leftFrontPos) * scale;
        double rightFrontAdjust = (maxPos - rightFrontPos) * scale;
        double leftBackAdjust = (maxPos - leftBackPos) * scale;
        double rightBackAdjust = (maxPos - rightBackPos) * scale;

        double leftFrontPower = (speed + leftFrontAdjust) * leftFrontSign;
        double rightFrontPower = (speed + rightFrontAdjust) * rightFrontSign;
        double leftBackPower = (speed + leftBackAdjust) * leftBackSign;
        double rightBackPower = (speed + rightBackAdjust) * rightBackSign;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (LOG_VERBOSE) {
            Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.3f %4.3f %4.3f %4.3f     position: %6.0f %6.0f %6.0f %6.0f",
                    leftFrontPower,
                    rightFrontPower,
                    leftBackPower,
                    rightBackPower,
                    leftFrontAdjust,
                    rightFrontAdjust,
                    leftBackAdjust,
                    rightBackAdjust,
                    leftFrontPos,
                    rightFrontPos,
                    leftBackPos,
                    rightBackPos);
        }
    }


    /**
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * @param direction direction to move
     * @param speed motor speed (-1 to 1)
     * @param inches  distance to move in inches, positive for forward, negative for backward
     * @param timeout timeout in milliseconds, 0 for no timeout
     */
    public void moveDistance(DIRECTION direction, double speed, double inches, double timeout) {

        int leftFrontSign = 0;
        int rightFrontSign = 0;
        int leftBackSign = 0;
        int rightBackSign = 0;

        if (direction == DIRECTION.FORWARD) {
            leftFrontSign  = 1;
            rightFrontSign = 1;
            leftBackSign   = 1;
            rightBackSign  = 1;
        } else if (direction == DIRECTION.BACK) {
            leftFrontSign  = -1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        } else if (direction == DIRECTION.RIGHT) {
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_LEFT) {
            leftFrontSign  = -1;
            rightFrontSign =  1;
            leftBackSign   = -1;
            rightBackSign  =  1;
        } else if (direction == DIRECTION.TURN_RIGHT){
            leftFrontSign  =  1;
            rightFrontSign = -1;
            leftBackSign   =  1;
            rightBackSign  = -1;
        }

        // Ensure that the OpMode is still active
        if (! opMode.opModeIsActive())  return;

        //Logger.message("moveDistance: heading %6.2f", getOrientation());

        DcMotor.RunMode mode = leftFrontDrive.getMode();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        int target = (int) (inches * encoderTicksPerInch());
        leftFrontDrive.setTargetPosition(target * leftFrontSign);
        rightFrontDrive.setTargetPosition(target * rightFrontSign);
        leftBackDrive.setTargetPosition(target * leftBackSign);
        rightBackDrive.setTargetPosition(target * rightBackSign);

        // Turn On RUN_TO_POSITION
        for (DcMotor motor : motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Looping until we move the desired distance
        elapsedTime.reset();
        boolean moving = true;
        while (opMode.opModeIsActive() && moving) {

            // Correct for drift
            double leftFrontPos = Math.abs(leftFrontDrive.getCurrentPosition());
            double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
            double leftBackPos = Math.abs(leftBackDrive.getCurrentPosition());
            double rightBackPos = Math.abs(rightBackDrive.getCurrentPosition());
            double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double speedRange = Math.max(Math.abs(speed) - RAMP_MIN_SPEED, 0);
            double ramUp = (elapsedTime.milliseconds() / RAMP_TIME) * speedRange + RAMP_MIN_SPEED;
            double ramDown = (Math.pow((Math.abs(target) - maxPos), 2) / Math.pow(RAMP_DISTANCE, 2)) * speedRange + RAMP_MIN_SPEED;
            double rampPower = Math.min(Math.min(ramUp, ramDown), speed);

            double scale = .0015;
            double leftFrontAdjust = (maxPos - leftFrontPos) * scale;
            double rightFrontAdjust = (maxPos - rightFrontPos) * scale;
            double leftBackAdjust = (maxPos - leftBackPos) * scale;
            double rightBackAdjust = (maxPos - rightBackPos) * scale;

            leftFrontDrive.setPower((rampPower + leftFrontAdjust) * leftFrontSign);
            rightFrontDrive.setPower((rampPower + rightFrontAdjust) * rightFrontSign);
            leftBackDrive.setPower((rampPower + leftBackAdjust) * leftBackSign);
            rightBackDrive.setPower((rampPower + rightBackAdjust) * rightBackSign);

            for (DcMotor motor : motors)
                if (! motor.isBusy())
                    moving = false;

            if (timeout > 0 && elapsedTime.milliseconds() >= timeout) {
                Logger.message("moveDistance: timed out");
                break;
            }

            if (LOG_VERBOSE)
                Logger.message("moveDistance: power: %4.2f %4.2f %4.2f %4.2f    adjust: %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     velocity: %4.2f %4.2f %4.2f %4.2f",
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontAdjust,
                        rightFrontAdjust,
                        leftBackAdjust,
                        rightBackAdjust,
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.getVelocity(),
                        rightFrontDrive.getVelocity(),
                        leftBackDrive.getVelocity(),
                        rightBackDrive.getVelocity());
        }

        // Stop all motion;
        for (DcMotor motor : motors)
            motor.setPower(0);

        // Restore run mode to prior state
        for (DcMotor motor : motors)
            motor.setMode(mode);

        Logger.message("%s  target  %6.2f  traveled %6.2f %6.2f %6.2f %6.2f  heading %6.2f  time %6.2f",
                direction,
                (double)target / encoderTicksPerInch(),
                (double)leftFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightFrontDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)leftBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                (double)rightBackDrive.getCurrentPosition() / encoderTicksPerInch(),
                getOrientation(),
                elapsedTime.seconds());
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
    public boolean moveToColor(COLOR color, double x, double y, double speed, double timeout){

        boolean found = false;
        float[] hsvValues = new float[3];
        ElapsedTime elapsedTime = new ElapsedTime();

        resetEncoders();
        moveRobot(x, y, 0, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive())  {
            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV color space
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];
            //Logger.message("hue %f saturation %f", hue, saturation);
            if (color == COLOR.BLUE) {
                if (hue >= 190 && hue <= 230 && saturation >= .5) {
                    Logger.message("blue line found");
                    found = true;
                }
            } else if (color == COLOR.RED) {
                //Logger.message("hue %f, saturation %f", hue, saturation);
                if ((hue >= 0 && hue <= 90) && saturation >= .5) {
                    Logger.message("red line found");
                    found = true;
                }
            }
            if (elapsedTime.milliseconds() > timeout){
                Logger.warning("no line found, traveled %5.2% inches", getDistanceTraveled());
                break;
            }
        }
        stopRobot();
        return found;
    }

    /**
     * Move forward until the distance sensor detects an object at the specified distance
     *
     * @param inches  distance for the object to stop
     * @param speed   speed to move forward
     * @param timeout timeout in milliseconds
     * @return true if an object was detected at the specified distance
     */
    public boolean moveToObject (double inches, double speed, double timeout) {
        boolean found = false;
        ElapsedTime elapsedTime = new ElapsedTime();

        resetEncoders();
        double startDistance = 0;
        double average;
        int count = 0;
        //moveRobot(1, 0, 0, speed);
        moveRobot(DIRECTION.FORWARD, speed);
        elapsedTime.reset();

        while (! found && opMode.opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            if (count == 0) {
                average = 0;
                startDistance = distance;
            } else {
                average = (startDistance - distance) / count;
                Logger.message("distance %6.2f  %6.2f", distance, average);
            }
            count++;

            if (distance - inches <= average / 2) {
                stopRobot();
                Logger.message("object found, distance %6.2f ", distance);
                found = true;
 //           } else if ( distance - inches < 2) {
 //               moveRobot(1, 0, 0, 0.1);
            }

            if (elapsedTime.milliseconds() > timeout){
                stopRobot();
                Logger.warning("no object found, traveled %6.2f inches", getDistanceTraveled());
                break;
            }
        }

        return found;
    }

    /**
     * Stop all the drive train motors.
     */
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        lastDirection = DIRECTION.STOOPED;
    }


    public void resetEncoders() {

        for (DcMotor motor : motors) {
            DcMotor.RunMode mode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(mode);
        }
    }

    public double encoderTicksPerInch() {
        return (COUNTS_PER_INCH * DRIVE_FACTOR);

    }
    public List<Double> getWheelPositions() {

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add((double)position / encoderTicksPerInch());
        }
        return wheelPositions;
    }

    public double getDistanceTraveled() {
        double traveled = 0;

        for (DcMotor motor : motors)
            traveled += motor.getCurrentPosition();
        return traveled / motors.size() / encoderTicksPerInch();
    }

    /**
     *  Return the current orientation of the robot.
     * @return orientation in a range of -180 to 180
     */
    public double getOrientation(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //Logger.message("Yaw   (Z) %6.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetOrientation() {
        imu.resetYaw();
    }

    public void forward (double distance) {
        moveDistance(DIRECTION.FORWARD,.4, distance, 0);
    }

    public void back (double distance) {

        moveDistance(DIRECTION.BACK,.4, distance, 0);
    }

    public void strafeLeft (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= STRAFE_FACTOR;
        moveDistance(DIRECTION.LEFT,.4, distance, 0);
    }

    public void strafeRight (double distance) {
        //distance = Math.sqrt(Math.pow(distance,2 ) + Math.pow(distance,2));
        distance *= STRAFE_FACTOR;
        moveDistance(DIRECTION.RIGHT,.4, distance, 0);
    }

    public void turn(double degrees) {

        double circumference = 2 * Math.PI * TURN_FACTOR;
        double inches = Math.abs(degrees) / 360 * circumference;
        if (degrees > 0)
            moveDistance(DIRECTION.TURN_LEFT, 0.4,  inches, 0 );
        else
            moveDistance(DIRECTION.TURN_RIGHT, 0.4,  inches, 0 );
    }

    public void turnWithIMU(double degrees) {
        imu.resetYaw();
        opMode.sleep(200);
        if (degrees > 0) {
            moveRobot(0, 0, 1, 0.3);
            while (opMode.opModeIsActive()) {
                double current = getOrientation();
                Logger.message("turn: degrees %6.1f  current %6.2f", degrees, current);
                if (current >= degrees-1)
                    break;
            }
        } else if (degrees < 0) {
            moveRobot(0, 0, -1, 0.3);
            while (opMode.opModeIsActive()) {
                double current = getOrientation();
                Logger.message("turn: degrees %6.1f  current %6.2f", degrees, current);
                if (current <= degrees+1)
                    break;
            }
        }
        stopRobot();
    }


} // end of class

