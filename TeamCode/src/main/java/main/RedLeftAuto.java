package main;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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

    private Trajectory traj1;
    private Trajectory traj2;
    private Trajectory traj3;
    private Trajectory traj4;
    private Trajectory traj5;

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (!robot.vision.findTeamElement()) {
            telemetry.addData("dir", "Left");
            traj1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(33.25)
                    .build();
            traj4 = drive.trajectoryBuilder(traj1.end())
                    .strafeLeft(14)
                    .build();
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj4);

            objectPosition = POSITION.left;

        } else {
            double angle = robot.vision.findTeamElementAngle();
            if (angle > -1) {
                telemetry.addData("dir", "Right");
                traj2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(33.25)
                        .build();
                traj5 = drive.trajectoryBuilder(traj2.end())
                        .strafeRight(21)
                        .build();
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj5);
            } else {
                telemetry.addData("dir", "Middle");
                traj3 = drive.trajectoryBuilder(new Pose2d())
                        .forward(33.25)
                        .build();
                drive.followTrajectory(traj3);
            }
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
