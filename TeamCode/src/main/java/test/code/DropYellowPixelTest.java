package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Auto;
import common.Robot;

@TeleOp(name="Drop Yellow Pixel Test", group="Test")
public class DropYellowPixelTest extends LinearOpMode {

    Robot robot = null;
    Auto auto;

    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        auto = new Auto(this, robot, null);

        waitForStart();

        auto.setColor(Auto.COLOR.RED);
        auto.dropYellowPixel();
        sleep(5000);
    }
}
