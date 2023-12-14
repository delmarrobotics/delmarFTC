package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Auto;
import common.Robot;

@TeleOp(name="Park Test", group="Tesr")
public class ParkTest extends LinearOpMode {

    private Robot robot = null;
    Auto auto;

    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        auto = new Auto(this, robot, null);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            auto.setColor(Auto.COLOR.RED);

            if (gamepad1.x) {
                auto.objectPosition = Auto.POSITION.left;
                auto.park();
            }
            if (gamepad1.a) {
                auto.objectPosition = Auto.POSITION.center;
                auto.park();
            }
            if (gamepad1.b) {
                auto.objectPosition = Auto.POSITION.right;
                auto.park();
            }
        }
    }
}
