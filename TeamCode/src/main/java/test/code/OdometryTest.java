package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
  private static final double MIN_SPEED = 0.20;
  private static final double TICKS_PER_REV = 2000;
  private static final double WHEEL_RADIUS = 0.944882;               // inches 48mm diameter

  static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI);

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

    driveWithOdometry(10);
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      telemetry.update();
      }
  }

  public void driveWithOdometry(double inches) {

    int leftPos = leftEncoder.getCurrentPosition();
    int rightPos = rightEncoder.getCurrentPosition();
    int frontPos = frontEncoder.getCurrentPosition();

    double targetPos = leftPos + (COUNTS_PER_INCH * inches);

    robot.moveRobot(1,0,0, 0.1);
    runtime.reset();
    while (runtime.milliseconds() > 3000) {
      int currentPos = leftEncoder.getCurrentPosition();
      telemetry.addData("positions",  "%f %d", targetPos, currentPos);
      if (targetPos >= currentPos) {
        break;
      }
      robot.stopRobot();
    }
  }
}
