package test.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

import common.Logger;
import common.Robot;
import common.Vision;
import main.RedLeftAuto;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Drive To April Tag", group="Test")
@SuppressWarnings("unused")
public class DriveToAprilTag extends LinearOpMode {

  private static final double MIN_SPEED = 0.20;
  private SampleMecanumDrive drive = null;

  private enum POSITION { left, center, right }
  DriveToAprilTag.POSITION objectPosition = POSITION.center;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Waiting for camera");
    telemetry.update();

    Robot robot = new Robot(this);
    robot.init();

    drive = new SampleMecanumDrive(hardwareMap);

    while (! robot.vision.cameraReady())
      sleep(100);
    sleep(1000);
    telemetry.addData("Status", "Press start");
    telemetry.update();


    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    //robot.moveToColor(Robot.COLOR.BLUE, 1, 0, MIN_SPEED, 4000);

    // run until the end of the match (driver presses STOP)
    boolean inPosition = false;
    while (opModeIsActive()) {
      if (robot.vision.findAprilTag(Vision.BLUE_CENTER_TAG)) {
        inPosition = true;
        telemetry.update();
        break;
      } else {
        Logger.message("no tag found");
        telemetry.update();
        break;
      }
    }

    if (inPosition) {
      double x = robot.vision.aprilTagX();
      Logger.message("x %f", x);
      double range = robot.vision.aprilTagY();
      Logger.message("range %f", range-4);

      robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.2,3000);
      /*
      Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
              .forward(range)
              .build();
      drive.followTrajectory(traj1);

       */

      double strafe;
      Trajectory traj2;
      if (objectPosition == POSITION.left) {
        traj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(5)
                .build();
        Logger.message("left");

      } else if (objectPosition == POSITION.center) {
        if (x > 0) {
          traj2 = drive.trajectoryBuilder(new Pose2d())
                  .strafeLeft(x)
                  .build();
          drive.followTrajectory(traj2);
        }
        else if (x < 0 ) {
          traj2 = drive.trajectoryBuilder(new Pose2d())
                  .strafeRight(-x)
                  .build();
          drive.followTrajectory(traj2);
        }
        Logger.message("center");

      } else {

        Logger.message("right");
      }
    }
  }
}


