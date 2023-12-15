/*
 * This file contains a OpMode for the autonomous phase when the robot starts at
 * the red left position.
 *
 */

package main;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import common.Auto;
import common.Robot;

@Autonomous(name="Red Left Start", group="Main")
public class RedLeftAuto extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    Auto.POSITION objectPosition;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the robot hardware.
        Robot robot = new Robot(this);
        robot.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Auto auto = new Auto(this, robot, drive);

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(24)
                .strafeLeft(12)
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .forward(21)
                .turn(Math.toRadians(-90))
                .forward(50)
                .strafeRight(30)
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(24)
                .turn(Math.toRadians(-90))
                .forward(12)
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .back(12)
                .strafeLeft(21)
                .forward(50)
                .strafeRight(30)
                .build();

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(27)
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .forward(18)
                .turn(Math.toRadians(-90))
                .forward(50)
                .strafeRight(30)
                .build();

        while (! robot.vision.cameraReady())
            sleep(100);
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        auto.setColor(Auto.COLOR.RED);
        objectPosition = auto.findTeamElement();

        if (objectPosition == Auto.POSITION.left) {
            robot.forward(25);
            robot.strafeLeft(13);
            robot.dropPurplePixel();
            robot.forward(24);
            robot.turn(-90);
            robot.forward(77+8.5);
            robot.strafeRight(24);

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(30);
            robot.dropPurplePixel();
            robot.forward(19);
            robot.turn(-90);
            robot.forward(77);
            robot.strafeRight(24);

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(24);
            robot.turn(90);
            robot.back(14);
            robot.dropPurplePixel();
            robot.forward(14);
            robot.turn(-90);
            robot.forward(25);
            robot.forward(77);
            robot.strafeRight(24);

        }
        auto.yellowPixel();

        telemetry.addData("Run Time", runtime.toString());
    }
}
