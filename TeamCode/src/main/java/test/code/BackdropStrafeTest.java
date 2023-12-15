package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Auto;
import common.Robot;

@TeleOp(name="Backdrop Strafe Test", group="Test")
public class BackdropStrafeTest extends LinearOpMode {

    private Robot robot = null;
    Auto auto;

    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        auto = new Auto(this, robot, null);

        while (! robot.vision.cameraReady())
            sleep(100);
        sleep(1000);


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            auto.setColor(Auto.COLOR.RED);

            if (gamepad1.x) {
                auto.objectPosition = Auto.POSITION.left;
                auto.strafeToDropPosition();
            }
            if (gamepad1.a) {
                auto.objectPosition = Auto.POSITION.center;
                auto.strafeToDropPosition();
            }
            if (gamepad1.b) {
                auto.objectPosition = Auto.POSITION.right;
                auto.strafeToDropPosition();
            }
        }
    }
}
