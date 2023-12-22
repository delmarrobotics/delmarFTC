package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import common.Logger;
import common.Robot;

/*
 * Test code for the hanging arm
 */

@TeleOp(name="Odometry Test", group="Test")
@SuppressWarnings("unused")
public class OdometryTest extends LinearOpMode {

  /*
  private static final double TICKS_PER_REV = 2000;
  private static final double WHEEL_RADIUS = 0.944882;               // inches 48mm diameter
  static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI);
   */

  static final double COUNTS_PER_MOTOR_REV = 28;              // HD Hex Motor Encoder
  static final double DRIVE_GEAR_REDUCTION = 20;              // Gearing
  static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // 96 mm while converted to inches
  static final double COUNTS_PER_INCH =
          (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


  private static final double DECELERATION_COUNT = 4 * COUNTS_PER_INCH;
  private static final double MAX_SPEED = 0.70;
  private static final double MIN_SPEED = 0.10;


  private final ElapsedTime runtime = new ElapsedTime();

  private Robot robot = null;

  private Encoder leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder;


  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot = new Robot(this);
    robot.init();

    robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    robot.rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    robot.leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    robot.rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    leftFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));   // left rear
    rightFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront")); // rightEncoder
    leftRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));     // frontEncoder
    rightRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));     // frontEncoder

    leftRearEncoder.setDirection(Encoder.Direction.REVERSE);
    rightRearEncoder.setDirection(Encoder.Direction.REVERSE);


    List<DcMotor> motors = Arrays.asList(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);


    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    //driveWithOdometry(0, 60, 0.3


    for (double power = 0.2; power < 0.5; power += 0.1) {

      for (DcMotor motor : motors) {
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
      }

      runtime.reset();
      moveRobotScaled(60, power);

      int leftFrontPos = leftFrontEncoder.getCurrentPosition();
      int rightFrontPos = rightFrontEncoder.getCurrentPosition();
      int leftRearPos = leftRearEncoder.getCurrentPosition();
      int rightRearPos = rightRearEncoder.getCurrentPosition();
      telemetry.addData("power", "%5.2f", power);
      telemetry.addData("left front",  "%d    %5.2f    %5.2f", leftFrontPos,  (leftFrontPos / COUNTS_PER_INCH),  (double)leftFrontPos / leftFrontPos);
      telemetry.addData("right front", "%d    %5.2f    %5.2f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), (double)rightFrontPos / leftFrontPos);
      telemetry.addData("left rear",   "%d    %5.2f    %5.2f", leftRearPos,   (leftRearPos / COUNTS_PER_INCH),   (double)leftRearPos / leftFrontPos);
      telemetry.addData("right rear",  "%d    %5.2f    %5.2f", rightRearPos,  (rightRearPos / COUNTS_PER_INCH),  (double)rightRearPos / leftFrontPos);
      telemetry.update();

      Logger.message("power %5.2f", power);
      Logger.message("time %5.2f", runtime.seconds());
      Logger.message("left front  %d    %5.2f    %5.2f", leftFrontPos,  (leftFrontPos / COUNTS_PER_INCH),  (double)leftFrontPos / leftFrontPos);
      Logger.message("right front %d    %5.2f    %5.2f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), (double)rightFrontPos / leftFrontPos);
      Logger.message("left rear   %d    %5.2f    %5.2f", leftRearPos,   (leftRearPos / COUNTS_PER_INCH),   (double)leftRearPos / leftFrontPos);
      Logger.message("right rear  %d    %5.2f    %5.2f", rightRearPos,  (rightRearPos / COUNTS_PER_INCH),  (double)rightRearPos / leftFrontPos);

      if  (! opModeIsActive()) break;
    }
  }


  private void moveRobotScaled (double inches, double power) {

    double current = 0;

    Encoder encoder = leftFrontEncoder;
    int start = encoder.getCurrentPosition();
    double distance = Math.abs(COUNTS_PER_INCH * inches);

    robot.leftFrontDrive.setPower(power);
    robot.rightFrontDrive.setPower(power);
    robot.leftBackDrive.setPower(power);
    robot.rightBackDrive.setPower(power);

    while (opModeIsActive()) {
      current = encoder.getCurrentPosition();
      if (Math.abs(current - start) >= distance) {
        robot.stopRobot();
        break;
      }
    }
    robot.stopRobot();
    Logger.message("moveRobotScaled: target %f  traveled %f", inches, current / COUNTS_PER_INCH);
  }


  /**
   * Move the robot by the specified inches forward, backward, left or right
   *
   * @param inchesX positive for forward, negative for backwards
   * @param inchesY positive for strafe left, negative for strafe right
   * @param speed 0-1 1 is maximum speed
   */
  public void driveWithOdometry(double inchesX, double inchesY, double speed) {

    Encoder encoder;
    double inches;
    double current = 0;
    double x = 0;
    double y = 0;

    if (inchesX != 0) {
      // forward or backward
      encoder = leftFrontEncoder;
      inches = inchesX;
      if (inchesX > 0)
        x = 1;
      else
        x = -1;

    } else if (inchesY != 0) {
      // left or right
      encoder = leftFrontEncoder;
      inches = inchesY;
      if (inchesY > 0)
        y = 1;
      else
        y = -1;

    } else {
      return;
    }

    int start = encoder.getCurrentPosition();
    double distance = Math.abs(COUNTS_PER_INCH * inches);

    robot.moveRobot(x, y,0, speed);

    while (opModeIsActive()) {
      current = encoder.getCurrentPosition();
      //Logger.message("traveled %f", Math.abs(current - start));
      if (Math.abs(current - start) >= distance) {
        robot.stopRobot();
        break;
      }
      double remaining = distance - Math.abs(current - start);
      if (remaining <= DECELERATION_COUNT) {
        double decelerationSpeed = (speed - MIN_SPEED) * (remaining / DECELERATION_COUNT);
        //robot.moveRobot(x, y,0, decelerationSpeed);
        //Logger.message("decelerationSpeed %f", decelerationSpeed);
      }
      if (remaining <= (COUNTS_PER_INCH * 2) ) {
        robot.moveRobot(x, y,0, 0.1);
      }
    }
    robot.stopRobot();
    Logger.message("driveWithOdometry: target %f  traveled %f", inches, current / COUNTS_PER_INCH);
  }
}
