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

public class Robot {
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    public DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    public DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    public DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    public ColorSensor colorSensor = null;
    public IMU imu = null;

    public HangingArm hangingArm = null;

    /* Declare OpMode members. */
    private HardwareMap hardwareMap;
    public LinearOpMode opMode;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode){
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        hangingArm = new HangingArm(opMode);

    }

    /**
     * Initialize the Robot
     */
    public void init() {

        initDriveTrain();

        // ToDo Check the the configuration file has the correct color sensor hardware device selected.
        colorSensor = hardwareMap.get(ColorSensor.class, Config.COLOR_SENSOR);

        imu = hardwareMap.get(IMU.class, Config.IMU);
    }

    /**
     * Initialize the drive train motors.
     */
    private void initDriveTrain(){
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, Config.LEFT_FRONT);
            rightFrontDrive = hardwareMap.get(DcMotor.class, Config.RIGHT_FRONT);
            leftBackDrive = hardwareMap.get(DcMotor.class, Config.LEFT_BACK);
            rightBackDrive = hardwareMap.get(DcMotor.class, Config.RIGHT_BACK);

        } catch (Exception e){
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
     * @param  x   Positive is forward
     * @param  y   Positive is strafe left
     * @param  yaw Positive Yaw is counter-clockwise
     */
    public void moveRobot (double x, double y, double yaw){

        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

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
     *  Move the robot forward or backward.
     *
     * @param inches distance to move in inches, positive for forward, negative for backward
     */
    public void moveByDistance(double inches) {
        // ToDo Use RobotAutoDriveByEncoder_Linear.java as an example to create this method
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
}
