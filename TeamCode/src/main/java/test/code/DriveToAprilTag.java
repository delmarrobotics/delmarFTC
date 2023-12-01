package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Logger;
import common.Robot;
import common.Vision;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Drive To April Tag", group="Test")
@SuppressWarnings("unused")
public class DriveToAprilTag extends LinearOpMode {

  private static final double MIN_SPEED = 0.20;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Press start");
    telemetry.update();

    Robot robot = new Robot(this);
    robot.init();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    //robot.moveToColor(Robot.COLOR.BLUE, 1, 0, MIN_SPEED, 4000);

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      boolean inPosition = false;
      do {
        if (robot.vision.findAprilTag(Vision.BLUE_RIGHT_TAG)) {
          double x = robot.vision.aprilTagX();
          Logger.message("x %f", x);
          if (x < 0 && x < -0.5) {
            robot.moveRobot(0, 1, 0, 0.2);
          } else if (x > 0 && x > 0.5) {
            robot.moveRobot(0, -1, 0, 0.2);
          } else {
            inPosition = true;
            robot.stopRobot();
          }
        } else {
          robot.stopRobot();
          break;
        }

        telemetry.update();
      } while (inPosition == false && opModeIsActive());

    }
  }
}
