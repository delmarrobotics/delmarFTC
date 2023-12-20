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
import common.Logger;
import common.Robot;

@Autonomous(name="Blue Right Start", group="Main")
public class BlueRightAuto extends LinearOpMode {

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
                .forward(8)
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .back(8)
                .turn(Math.toRadians(90))
                .forward(23.5)
                .turn(Math.toRadians(90))
                .forward(70.5)
                .strafeLeft(29.5)
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(30)
                .turn(Math.toRadians(-110))
                .forward(6.5)
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .forward(6.5)
                .turn(Math.toRadians(110))
                .forward(23.5)
                .turn(Math.toRadians(110))
                .forward(90)
                .strafeLeft(29.5)
                .build();

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(31.25)
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .forward(23.5)
                .turn(Math.toRadians(90))
                .forward(70.5)
                .strafeLeft(23.5)
                .build();


        telemetry.addLine("waiting for camera");
        telemetry.update();
        while (! robot.vision.cameraReady())
            sleep(100);
        telemetry.addLine("camera ready, press start");
        telemetry.update();

        robot.vision.enableCameraStream(true);    // ToDo for debugging

        waitForStart();
        runtime.reset();

        auto.setColor(Auto.COLOR.BLUE);
        objectPosition = auto.findTeamElement();

        if (objectPosition == Auto.POSITION.left) {
            robot.forward(24);
            robot.turn(90);
            robot.forward(6);
            robot.dropPurplePixel();
            robot.back(10);
            robot.turn(-90);
            robot.forward(25);
            robot.turn(90);
            robot.forward(67+4);
            robot.strafeLeft(24);

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(30);
            robot.dropPurplePixel();
            robot.forward(19);
            robot.turn(90);
            robot.forward(67);
            robot.strafeLeft(24);

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(25);
            robot.strafeRight(8.5);
            robot.dropPurplePixel();
            robot.forward(24);
            robot.turn(90);
            robot.forward(67+8.5);
            robot.strafeLeft(24);
        }

        auto.strafeToDropPosition();
        auto.dropYellowPixel();
        auto.parkCenter();

        Logger.message("Run Time %s", runtime.toString());
    }
}
