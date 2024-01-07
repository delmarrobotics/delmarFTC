package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Auto;
import common.Robot;

@TeleOp(name="Backdrop Strafe Test", group="Test")
//@Disabled
public class BackdropStrafeTest extends LinearOpMode {

    private Robot robot = null;
    Auto auto;

    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        auto = new Auto(this, robot);

        telemetry.addLine("Waiting for camera");
        telemetry.update();
        while (! robot.vision.cameraReady())
            sleep(100);
        telemetry.addLine("Camera ready");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            auto.setColor(Auto.COLOR.BLUE);

            if (gamepad1.x) {
                auto.objectPosition = Auto.POSITION.left;
                auto.adjustYaw();
                auto.strafeToDropPosition();
                auto.dropYellowPixel();
            }
            if (gamepad1.a) {
                auto.objectPosition = Auto.POSITION.center;
                auto.adjustYaw();
                auto.strafeToDropPosition();
                auto.dropYellowPixel();
            }
            if (gamepad1.b) {
                auto.objectPosition = Auto.POSITION.right;
                auto.adjustYaw();
                auto.strafeToDropPosition();
                auto.dropYellowPixel();
            }
            if (gamepad1.y) {
                robot.vision.findAprilTag(-1);
                while (gamepad1.y) sleep(100);
            }
        }
    }
}
