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

        Auto auto = new Auto(this, robot);

        telemetry.addLine("waiting for camera");
        telemetry.update();

        while (!robot.vision.cameraReady())
            sleep(100);
        telemetry.addLine("camera ready, press start"); //sleep(200) not needed here
        telemetry.update();

        robot.vision.enableCameraStream(true);

        waitForStart();
        runtime.reset();

        robot.vision.enableCameraStream(false);

        auto.setColor(Auto.COLOR.BLUE);
        objectPosition = auto.findTeamElement();

        if (objectPosition == Auto.POSITION.left) {
            robot.forward(12);
            robot.strafeLeft(13);
            robot.forward(29-12);
            robot.dropPurplePixel();
            robot.back(8);
            robot.turn(90);
            robot.forward(23.5-13);
            robot.strafeRight(4);

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(33);
            robot.dropPurplePixel();
            robot.back(8);
            robot.turn(90);
            robot.forward(23.5);

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(26.6);
            robot.turn(90);
            robot.back(14);
            robot.dropPurplePixel();
            robot.forward(23.5+14);
        }

        robot.vision.enableCameraStream(true);
        auto.strafeToDropPosition();
        auto.dropYellowPixel();
        auto.parkCorner();
    }
}
