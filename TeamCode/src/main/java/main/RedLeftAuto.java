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
            traj1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(30)
                    .build();
            traj2 = drive.trajectoryBuilder(traj1.end())
                    .strafeLeft(20)
                    .build();
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

            //Todo save position of object
        } else {
            double angle = robot.vision.findTeamElementAngle();
            if (angle > 5) {

            } else {

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
