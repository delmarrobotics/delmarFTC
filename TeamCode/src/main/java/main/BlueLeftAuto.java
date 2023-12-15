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

import org.checkerframework.checker.propkey.qual.PropertyKeyBottom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import common.Auto;
import common.PixelArm;
import common.Robot;

@Autonomous(name="Blue Left Start", group="Main")
public class BlueLeftAuto extends LinearOpMode {

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
                .forward(31.25)
                .turn(Math.toRadians(90))
                .forward(10)
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .forward(26.5)
                .strafeLeft(10)
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(27.25)
                .turn(Math.toRadians(-90))
                .forward(8)
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .back(8)
                .turn(Math.toRadians(-180))
                .forward(36.5)
                .strafeRight(6)
                .build();

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(31.25)
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .back(6)
                .turn(Math.toRadians(90))
                .forward(36.5)
                .build();

        telemetry.addLine("waiting for camera");
        telemetry.update();
        while (! robot.vision.cameraReady())
            sleep(100);
        sleep(1000);
        telemetry.addLine("camera ready");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        auto.setColor(Auto.COLOR.BLUE);
        objectPosition = auto.findTeamElement();

        if (objectPosition == Auto.POSITION.left) {
            robot.forward(12);
            robot.strafeLeft(13);
            robot.forward(13);
            robot.dropPurplePixel();
            robot.back(8);
            robot.turn(90);
            robot.forward(15);
            robot.strafeRight(1);
            auto.dropYellowPixel();
            auto.park();

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(30.5);
            robot.dropPurplePixel();
            robot.back(8);
            robot.turn(90);
            robot.forward(26);
            robot.strafeRight(3);
            auto.dropYellowPixel();
            auto.park();

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(24);
            robot.turn(90);
            robot.back(14);
            robot.dropPurplePixel();
            robot.forward(40);
            robot.strafeRight(6);
        }

        auto.dropYellowPixel();
        auto.parkCenter();

        telemetry.addData("Run Time", runtime.toString());
    }
}
