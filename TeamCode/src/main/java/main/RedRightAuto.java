/*
 * This file contains a OpMode for the autonomous phase when the robot starts at
 * the red left position.
 *
 */

package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        Auto auto = new Auto(this, robot);

        telemetry.addLine("waiting for camera");
        telemetry.update();
        auto.waitForCamera();
        telemetry.addLine("camera ready, press start");
        telemetry.update();
        
        robot.vision.enableCameraStream(true);

        waitForStart();
        runtime.reset();

        robot.vision.enableCameraStream(false);

        auto.setColor(Auto.COLOR.RED);
        objectPosition = auto.findTeamElement();

        if (objectPosition == Auto.POSITION.left) {
            robot.forward(26.5);
            robot.turn(90);
            robot.forward(6.5);
            robot.dropPurplePixel();
            robot.back(6.5+23.5);
            robot.turn(180);

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(32);
            robot.dropPurplePixel();
            robot.back(8);
            robot.turn(-90);
            robot.forward(23.5);

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(12);
            robot.strafeRight(8.5);
            robot.forward(29-12);
            robot.dropPurplePixel();
            robot.back(8);
            robot.turn(-90);
            robot.forward(23.5-8.5);
        }

        robot.vision.enableCameraStream(true);
        auto.strafeToDropPosition();
        auto.dropYellowPixel(Auto.DROP_POSITION.LOW);
        auto.parkCorner();
    }
}
