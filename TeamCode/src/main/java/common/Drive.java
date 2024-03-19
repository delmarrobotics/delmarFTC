package common;

import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import java.util.List;

public class Drive extends Thread {

    public static double DRIVE_FACTOR  = 0.94;      // 1;
    public static double STRAFE_FACTOR = 1.2;       // 1.11;
    public static double TURN_FACTOR   = 13.38;     // (24.9/2);

    // Drive train
    private final double COUNTS_PER_MOTOR_REV = 28.065;          // Gobilda 5203 motor (384.5 motor rev / 13.7 gear ratio)
    private final double DRIVE_GEAR_REDUCTION = 13.7;            // Gearing
    private final double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches, TODO: measure new wheel size
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final double RAMP_DISTANCE = COUNTS_PER_INCH * 12;   // ramp down distance in encoder counts
    private final double RAMP_TIME = 1000;                       // ramp up time in milliseconds
    private final double RAMP_MIN_SPEED = 0.2;

    private final double MIN_SPEED = 0.25;
    private final double MAX_SPEED = 0.9;
    private final double MAX_ROTATE_SPEED = 0.50;

    // Color sensor
    static final float COLOR_SENSOR_GAIN = 2.2F;

    public enum COLOR {RED, BLUE}

    public DcMotorEx leftFrontDrive = null;   //  Used to control the left front drive wheel
    public DcMotorEx rightFrontDrive = null;  //  Used to control the right front drive wheel
    public DcMotorEx leftBackDrive = null;    //  Used to control the left back drive wheel
    public DcMotorEx rightBackDrive = null;   //  Used to control the right back drive wheel

    public Encoder sideEncoder;
    private IMU imu;
    private NormalizedColorSensor colorSensor = null;

    List<DcMotorEx> motors;
    LinearOpMode opMode;

    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
        this.setName("Drive");
        init();
    }

    public void init() {

    }

    public void loop() {

    }
}
