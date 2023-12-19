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

        Auto auto = new Auto(this, robot, null);

        telemetry.addLine("waiting for camera");
        telemetry.update();

        while (!robot.vision.cameraReady())
            sleep(100);
        sleep(200);    // ToDo needed?

        telemetry.addLine("camera ready, press start");
        telemetry.update();

        robot.vision.enableCameraStream(true);    // ToDo for debugging

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {

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

            } else if (objectPosition == Auto.POSITION.center) {
                robot.forward(30.5);
                robot.dropPurplePixel();
                robot.back(8);
                robot.turn(90);
                robot.forward(25);
                robot.strafeRight(3);

            } else if (objectPosition == Auto.POSITION.right) {
                robot.forward(24);
                robot.turn(90);
                robot.back(14);
                robot.dropPurplePixel();
                robot.forward(38);
                robot.strafeRight(6);
            }

            auto.strafeToDropPosition();
            auto.dropYellowPixel();
            auto.parkCorner();

            telemetry.addData("Run Time", runtime.toString());
        }
    }
}
