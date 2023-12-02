package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import common.Logger;
import common.Robot;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Odometry Test", group="Test")
@SuppressWarnings("unused")
public class OdometryTest extends LinearOpMode {

  private static final double MAX_SPEED = 0.70;
  private static final double MIN_SPEED = 0.10;
  private static final double TICKS_PER_REV = 2000;
  private static final double WHEEL_RADIUS = 0.944882;               // inches 48mm diameter
  static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI);
  private static final double DECELERATION_COUNT = 4 * COUNTS_PER_INCH;


  private final ElapsedTime runtime = new ElapsedTime();

  private Robot robot = null;

  private Encoder leftEncoder, rightEncoder, frontEncoder;


  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot = new Robot(this);
    robot.init();

    leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));   // left rear
    rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront")); // rightEncoder
    frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));  // frontEncoder
    leftEncoder.setDirection(Encoder.Direction.REVERSE);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    driveWithOdometry(0, 10, 0.2);

    while (opModeIsActive()) {

      int leftPos = leftEncoder.getCurrentPosition();
      int rightPos = rightEncoder.getCurrentPosition();
      int frontPos = frontEncoder.getCurrentPosition();
      telemetry.addData("positions", "left %d   right %d  front %d",
          leftPos, rightPos, frontPos);
      telemetry.update();
      }
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
      encoder = leftEncoder;
      inches = inchesX;
      if (inchesX > 0)
        x = 1;
      else
        x = -1;

    } else if (inchesY != 0) {
      // left or right
      encoder = frontEncoder;
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
        Logger.message("decelerationSpeed %f", decelerationSpeed);
      }
      if (remaining <= (COUNTS_PER_INCH * 2) ) {
        robot.moveRobot(x, y,0, 0.1);
      }
    }
    robot.stopRobot();
    Logger.message("driveWithOdometry: target %f  traveled %f", inches, current / COUNTS_PER_INCH);
  }
}
