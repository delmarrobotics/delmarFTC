package test.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

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
  private SampleMecanumDrive drive = null;

  private enum POSITION { left, center, right }
  DriveToAprilTag.POSITION objectPosition = POSITION.right;

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


    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      if (robot.vision.findAprilTag(Vision.BLUE_RIGHT_TAG)) { //ToDo change to left
        double x = robot.vision.aprilTagX();
        double range = robot.vision.aprilTagY();
        double yaw = robot.vision.aprilTagYaw();
        Logger.message("x %f  range %f  yaw %f", x, range, yaw);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(yaw))
                //.forward(range-2)
                .build();
        drive.followTrajectorySequence(traj1);

        Logger.message("robot orientation %3.1f", robot.getOrientation());

        //robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.2,3000);
        //robot.moveToColor(Robot.COLOR.BLUE, 1, 0, MIN_SPEED, 4000);

        double strafe = 0;
        if (objectPosition == POSITION.left) {
          strafe = x - 6;
          Logger.message("left, strafe %f", strafe);
        } else if (objectPosition == POSITION.center) {
          strafe = x;
          Logger.message("center, strafe %f", strafe);
        } else {
          strafe = 6 + x;
          Logger.message("right, strafe %f", strafe);
        }

        Trajectory traj2;
        if (strafe > 0) {
          traj2 = drive.trajectoryBuilder(new Pose2d())
                  .strafeRight(x)
                  .build();
          drive.followTrajectory(traj2);
        }
        else if (strafe < 0 ) {
          traj2 = drive.trajectoryBuilder(new Pose2d())
                  .strafeLeft(-strafe)
                  .build();
          drive.followTrajectory(traj2);
        }
        break;

      } else {
        Logger.message("Tag not found");
      }
    }
  }
}


