package main;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import common.Logger;
import common.Robot;

/*
 * This file contains a OpMode for the autonomous
 *
 */

@Autonomous(name="Red Left Start", group="Main")  // ToDo change to @Autonomous when testing is complete
public class RedLeftAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = null;
    private SampleMecanumDrive drive = null;

    private TrajectorySequence traj1;
    private TrajectorySequence traj2;
    private TrajectorySequence traj3;

    private enum POSITION { left, center, right }
    POSITION objectPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

         drive = new SampleMecanumDrive(hardwareMap);

        while (! robot.vision.cameraReady())
            sleep(100);
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (!robot.vision.findTeamElement(3000)) {
            // Team element not found
            objectPosition = POSITION.right;
            telemetry.addData("dir", "Right");
            Logger.message("Team element at right position");
        } else {
            // Team element is on the left spike mark
            double angle = robot.vision.findTeamElementAngle();
            objectPosition = POSITION.left;
            if (angle < 0) {
                telemetry.addData("dir", "Left");
                Logger.message("Team element at left position, angle %f", angle);
            } else {
                // Team element is on the center spike mark
                telemetry.addData("dir", "Middle");
                Logger.message("Team element at center position, angle %f", angle);
                objectPosition = POSITION.center;
            }
        }

        objectPosition = POSITION.center;

        if (objectPosition == POSITION.left) {
            traj1 = drive.trajectorySequenceBuilder(new Pose2d(-35.25, -62.5, Math.toRadians(90)))
                    .strafeLeft(13)
                    .lineTo(new Vector2d(-48.25, -31))
                    .waitSeconds(0.5)
                    .addDisplacementMarker(() ->robot.dropPurplePixel())
                    .lineTo(new Vector2d(-48.25, -11.75))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d( 11.75, -11.75))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(11.75, -29.25))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(47.75,-29.25))
                    .build();
            drive.followTrajectorySequence(traj1);

        } else if (objectPosition == POSITION.right) {
            traj2 = drive.trajectorySequenceBuilder(new Pose2d(-35.25, -62.5, Math.toRadians(90)))
                    .lineTo(new Vector2d(-35.25, -33))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(-30,-33))
                    .waitSeconds(0.5)
                    .addDisplacementMarker(() ->robot.dropPurplePixel())
                    .lineTo(new Vector2d(47.25, -41.25))
                    .build();
            drive.followTrajectorySequence(traj2);

        } else if (objectPosition == POSITION.center) {
            traj3 = drive.trajectorySequenceBuilder(new Pose2d(-35.25, -62.5, Math.toRadians(90)))
                    .lineTo(new Vector2d(-35.25, -29))
                    .waitSeconds(2.5)
                    .addDisplacementMarker(() -> robot.dropPurplePixel())
                    //.lineTo(new Vector2d(-35.25, -11.75))
                    //.turn(Math.toRadians(-90))
                    //.lineTo(new Vector2d( 11.75, -11.75))
                    //.turn(Math.toRadians(-90))
                    //.lineTo(new Vector2d(11.75, -35.25))
                    //.turn(Math.toRadians(90))
                    //.lineTo(new Vector2d(47.75,-35.25))
                    .build();
            drive.followTrajectorySequence(traj3);
        }



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
