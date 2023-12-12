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

@Autonomous(name="Red Right Start", group="Main")
public class RedRightAuto extends LinearOpMode {

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
                .forward(27.25)
                .turn(Math.toRadians(-90))
                .back(15.75)
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .forward(15.75)
                .turn(Math.toRadians(90))
                .forward(23.5)
                .turn(Math.toRadians(90))
                .forward(70.5)
                .strafeLeft(29.5)
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(27.25)
                .turn(Math.toRadians(-90))
                .forward(8)
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .back(8)
                .turn(Math.toRadians(90))
                .forward(23.5)
                .turn(Math.toRadians(90))
                .forward(70.5)
                .strafeLeft(17.5)
                .build();

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(33.5)
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .forward(17.25)
                .turn(Math.toRadians(90))
                .forward(70.5)
                .strafeLeft(23.5)
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
            robot.forward(27);
            robot.turn(90);
            robot.forward(7);
            sleep(500);
            robot.dropPurplePixel();
            robot.back(12);
            robot.turn(90);
            robot.turn(90);
            robot.forward(15);

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(34);
            robot.turn(-90);
            robot.forward(6.5);
            sleep(500);
            robot.dropPurplePixel();
            robot.forward(15);

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(35);
            sleep(500);
            robot.dropPurplePixel();
            robot.back(12);
            robot.turn(-90);
            robot.forward(21.5);
        }

        auto.yellowPixel();

        telemetry.addData("Run Time", runtime.toString());
    }
}
