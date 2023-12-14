package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Robot;

@TeleOp(name="Route", group="Test")
public class RouteTest extends LinearOpMode {

    Robot robot = null;
    private final ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        waitForStart();

        double inches = 12;
        while (opModeIsActive()) {

            telemetry.addData("inches", " %4.2f", inches);
            telemetry.update();

            if (gamepad1.y) {
                robot.forward(inches);
            }
            if (gamepad1.a) {
                robot.back(inches);
            }
            if (gamepad1.x) {
                robot.strafeLeft(inches);
            }
            if (gamepad1.b) {
                robot.strafeRight(inches);
            }
            if (gamepad1.dpad_left){
                robot.turn(90);
            }
            if (gamepad1.dpad_right) {
                robot.turn(-90);
            }
            if (gamepad1.left_bumper) {
                runtime.reset();
                while (gamepad1.left_bumper) {
                    inches -= (double) increment(.5, 1, 5);
                    telemetry.addData("inches", " %4.2f", inches);
                    telemetry.update();
                }
            }
            if (gamepad1.right_bumper) {
                runtime.reset();
                while (gamepad1.left_bumper) {
                    inches += (double) increment(.5, 1, 5);
                    telemetry.addData("inches", " %4.2f", inches);
                    telemetry.update();
                }
            }
        }
    }

    /* Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    private double increment(double v1, double v2, double v3){
        long sleepTime;
        double delta;
        if (runtime.seconds() < 3){
            delta = v1;
            sleepTime = 500;
        }
        else if (runtime.seconds() < 6){
            delta = v2;
            sleepTime = 200;
        }
        else{
            delta = v3;
            sleepTime = 100;
        }
        sleep(sleepTime);
        return delta;
    }

}
