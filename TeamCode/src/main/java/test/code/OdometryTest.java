/*
 * Odometer test
 */
package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;


import common.Logger;
import common.Robot;


@TeleOp(name="Odometry Test", group="Test")
@Disabled
@SuppressWarnings("unused")
public class OdometryTest extends LinearOpMode {


  private static final double ODOMETER_TICKS_PER_REV = 2000;
  private static final double ODOMETER_WHEEL_DIAMETER = 1.90278;  //  1.92913;       // 0.9863 in inches, 48mm diameter
  private static final double ODOMETER_COUNTS_PER_INCH = ODOMETER_TICKS_PER_REV / (ODOMETER_WHEEL_DIAMETER * Math.PI);

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

  private DcMotorEx sideEncoderMotor;
  private Encoder leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder, sideEncoder;


  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot = new Robot(this);
    robot.init();

    leftFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));   // left rear
    rightFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront")); // rightEncoder
    leftRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));     // frontEncoder
    rightRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));     // frontEncoder

    leftRearEncoder.setDirection(Encoder.Direction.REVERSE);
    rightRearEncoder.setDirection(Encoder.Direction.REVERSE);

    sideEncoderMotor = hardwareMap.get(DcMotorEx.class, "sideEncoder");
    sideEncoder = new Encoder(sideEncoderMotor);

    waitForStart();

    distanceTest();

    /*
    sideEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    while (opModeIsActive()) {
      int position = sideEncoder.getCurrentPosition();
      Logger.message("encoder position %d  %8.2f", position, (double)position/ODOMETER_COUNTS_PER_INCH);
      telemetry.addData("encoder position","%d  %8.2f", position, (double)position/ODOMETER_COUNTS_PER_INCH);
      telemetry.update();
      sleep(250);
    }
    */
  }

  private void distanceTest() {

    robot.drive.resetEncoders();
    sideEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    runtime.reset();

    robot.forward(60);
    sleep(2000);
    displayResults();

    while (opModeIsActive()) {
      sleep(100);
    }
  }


  private void displayResults() {

    int leftFrontPos = leftFrontEncoder.getCurrentPosition();
    int rightFrontPos = rightFrontEncoder.getCurrentPosition();
    int leftRearPos = leftRearEncoder.getCurrentPosition();
    int rightRearPos = rightRearEncoder.getCurrentPosition();
    int odometerPos = sideEncoder.getCurrentPosition();

    telemetry.addData("odometer",   "%6d    %5.2f", odometerPos,  (odometerPos  / ODOMETER_COUNTS_PER_INCH));
    telemetry.addData("left front", "%6d    %5.2f    %6.3f", leftFrontPos,  (leftFrontPos  / COUNTS_PER_INCH), (((double)leftFrontPos  / leftFrontPos) - 1) * 100);
    telemetry.addData("right front","%6d    %5.2f    %6.3f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), (((double)rightFrontPos / leftFrontPos) - 1) * 100);
    telemetry.addData("left rear",  "%6d    %5.2f    %6.3f", leftRearPos,   (leftRearPos   / COUNTS_PER_INCH), (((double)leftRearPos   / leftFrontPos) - 1) * 100);
    telemetry.addData("right rear", "%6d    %5.2f    %6.3f", rightRearPos,  (rightRearPos  / COUNTS_PER_INCH), (((double)rightRearPos  / leftFrontPos) - 1) * 100);
    telemetry.update();

    Logger.message("odometer    %6d    %5.2f", odometerPos,  (odometerPos  / ODOMETER_COUNTS_PER_INCH));
    Logger.message("left front  %6d    %5.2f    %6.3f", leftFrontPos,  (leftFrontPos / COUNTS_PER_INCH),  (((double)leftFrontPos  / leftFrontPos) - 1) * 100);
    Logger.message("right front %6d    %5.2f    %6.3f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), (((double)rightFrontPos / leftFrontPos) - 1) * 100);
    Logger.message("left rear   %6d    %5.2f    %6.3f", leftRearPos,   (leftRearPos / COUNTS_PER_INCH),   (((double)leftRearPos   / leftFrontPos) - 1) * 100);
    Logger.message("right rear  %6d    %5.2f    %6.3f", rightRearPos,  (rightRearPos / COUNTS_PER_INCH),  (((double)rightRearPos  / leftFrontPos) - 1) * 100);
    Logger.message("\n");

  }

  private void moveRobotScaled (double inches, double power) {

    double current = 0;

    Encoder encoder = leftFrontEncoder;
    int start = encoder.getCurrentPosition();
    double distance = Math.abs(COUNTS_PER_INCH * inches);

    robot.drive.leftFrontDrive.setPower(power);
    robot.drive.rightFrontDrive.setPower(power);
    robot.drive.leftBackDrive.setPower(power);
    robot.drive.rightBackDrive.setPower(power);

    while (opModeIsActive()) {
      current = encoder.getCurrentPosition();
      if (Math.abs(current - start) >= distance) {
        robot.drive.stopRobot();
        break;
      }
    }
    robot.drive.stopRobot();
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

    robot.drive.moveRobot(x, y,0, speed);

    while (opModeIsActive()) {
      current = encoder.getCurrentPosition();
      //Logger.message("traveled %f", Math.abs(current - start));
      if (Math.abs(current - start) >= distance) {
        robot.drive.stopRobot();
        break;
      }
      double remaining = distance - Math.abs(current - start);
      if (remaining <= DECELERATION_COUNT) {
        double decelerationSpeed = (speed - MIN_SPEED) * (remaining / DECELERATION_COUNT);
        //robot.moveRobot(x, y,0, decelerationSpeed);
        //Logger.message("decelerationSpeed %f", decelerationSpeed);
      }
      if (remaining <= (COUNTS_PER_INCH * 2) ) {
        robot.drive.moveRobot(x, y,0, 0.1);
      }
    }
    robot.drive.stopRobot();
    Logger.message("driveWithOdometry: target %f  traveled %f", inches, current / COUNTS_PER_INCH);
  }
}
