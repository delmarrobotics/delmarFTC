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
  DriveToAprilTag.POSITION objectPosition = POSITION.left;

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


    Trajectory traj = drive.trajectoryBuilder(new Pose2d())
            .strafeRight(5)
            .build();
    drive.followTrajectory(traj);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    //robot.moveToColor(Robot.COLOR.BLUE, 1, 0, MIN_SPEED, 4000);

    // run until the end of the match (driver presses STOP)
    boolean inPosition = false;
    while (opModeIsActive()) {
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
          break;
        }
      } else {
        Logger.message("no tag found");
        robot.stopRobot();
        break;
      }
      telemetry.update();
    }

    if (inPosition) {
      double range = robot.vision.aprilTagY();
      Logger.message("range %f", range);
      Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
              .forward(range)
              .build();
      drive.followTrajectory(traj1);

      double strafe;
      Trajectory traj2;
      if (objectPosition == POSITION.left) {
        traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(5)
                .build();
        Logger.message("left");
      } else if (objectPosition == POSITION.center) {
        traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(1)
                .build();
        Logger.message("center");
      } else {
        traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(7)
                .build();
        Logger.message("right");
      }
      drive.followTrajectory(traj2);
    }
  }
}


