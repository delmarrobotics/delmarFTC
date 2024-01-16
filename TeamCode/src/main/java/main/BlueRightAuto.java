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

        Auto auto = new Auto(this, robot);

        telemetry.addLine("waiting for camera");
        telemetry.update();
        while (! robot.vision.cameraReady())
            sleep(100);
        telemetry.addLine("camera ready, press start");
        telemetry.update();

        robot.vision.enableCameraStream(true);

        waitForStart();
        runtime.reset();

        robot.vision.enableCameraStream(false);

        auto.setColor(Auto.COLOR.BLUE);
        objectPosition = auto.findTeamElement();

        if (objectPosition == Auto.POSITION.left) {
            robot.forward(26.5);
            robot.turn(90);
            robot.forward(7.5);
            robot.dropPurplePixel();
            robot.back(7.5);
            robot.turn(-90);
            robot.forward(23.5);
            robot.turn(90);
            robot.forward(70);
            robot.strafeLeft(24);

        } else if (objectPosition == Auto.POSITION.center) {
            robot.forward(32);
            robot.dropPurplePixel();
            robot.forward(18);
            robot.turn(90);
            robot.forward(70);
            robot.strafeLeft(24);

        } else if (objectPosition == Auto.POSITION.right) {
            robot.forward(26.5);
            robot.strafeRight(8.5);
            robot.dropPurplePixel();
            robot.forward(23.5);
            robot.turn(90);
            robot.forward(70+8.5);
            robot.strafeLeft(24);
        }

        auto.strafeToDropPosition();
        auto.dropYellowPixel(Auto.DROP_POSITION.HIGH);
        auto.parkCenter();

        Logger.message("Run Time %s", runtime.toString());
    }
}
