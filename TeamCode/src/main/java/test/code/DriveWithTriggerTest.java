package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.HangingArm;
import common.Logger;
import common.Robot;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Drive with Trigger Test", group="Test")
@SuppressWarnings("unused")
public class DriveWithTriggerTest extends LinearOpMode {

  private static final double MAX_SPEED = 0.5;
  private static final double MIN_SPEED = 0.1;

  private final ElapsedTime runtime = new ElapsedTime();

  private Robot robot = null;


  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot = new Robot(this);
    robot.init();


    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // POV Mode uses left stick to go forward, and right stick to rotate.
      double drive  = -gamepad1.left_stick_y;
      double strafe = -gamepad1.left_stick_x;
      double yaw   = -gamepad1.right_stick_x;
      double speed = gamepad1.left_trigger;
      moveRobot(drive, strafe, yaw, speed);

      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.update();
      }
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

        if (speed < MIN_SPEED)
          speed = MIN_SPEED;
        else if (speed > MAX_SPEED) {
          speed = MAX_SPEED;
        }
      double scale = (1 / max) * speed;

        leftFrontPower *= scale;
        rightFrontPower *= scale;
        leftBackPower *= scale;
        rightBackPower *= scale;

        Logger.message("power %f %f %f %f %f", speed, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // Send powers to the wheels.
    robot.leftFrontDrive.setPower(leftFrontPower);
    robot.rightFrontDrive.setPower(rightFrontPower);
    robot.leftBackDrive.setPower(leftBackPower);
    robot.rightBackDrive.setPower(rightBackPower);
  }
}
