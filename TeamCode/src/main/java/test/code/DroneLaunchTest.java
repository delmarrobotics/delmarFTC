package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Robot;

@TeleOp(name="Drone Launch Test", group="Test")
public class DroneLaunchTest extends LinearOpMode {

    Robot robot = null;
    private final ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        telemetry.addLine("push start");
        telemetry.update();

        waitForStart();

        telemetry.setAutoClear(false);
        telemetry.addData("Controls", "\n" +
                "  right trigger - launch drone\n" +
                "  left trigger  - launch down position\n" +
                "  left bumper - decrease launch position\n" +
                "  right bumper - increase launcg position\n" +
                "\n" +
                "current drone launch position %6.3f\n", robot.DRONE_ANGLE_UP);

        Telemetry.Item positionMsg = telemetry.addData("drone launch position", " %6.3f", robot.DRONE_ANGLE_UP);
        telemetry.update();

        while (opModeIsActive()) {

            //telemetry.addData("drone launch position", " %4.2f", robot.DRONE_ANGLE_UP);
            //telemetry.update();

            if (gamepad1.right_trigger > 0) {
                // Launch the drone
                robot.droneAngle.setPosition(robot.DRONE_ANGLE_UP);
                sleep(1000);
                robot.droneFire.setPosition(robot.DRONE_FIRE_UP);
                while (gamepad1.right_trigger > 0) sleep(100);
            }

            if (gamepad1.left_trigger > 0) {
                robot.droneFire.setPosition(robot.DRONE_FIRE_DOWN);
                robot.droneAngle.setPosition(robot.DRONE_ANGLE_DOWN);
                while (gamepad1.left_trigger > 0) sleep(100);
            }

            if (gamepad1.left_bumper) {
                // angle the drone down
                robot.DRONE_ANGLE_UP = Math.min(robot.DRONE_ANGLE_UP + 0.001, 1);
                positionMsg.setValue( " %6.3f", robot.DRONE_ANGLE_UP);
                telemetry.update();
                while (gamepad1.left_bumper) sleep(100);
            }

            if (gamepad1.right_bumper) {
                // angle the drone up
                robot.DRONE_ANGLE_UP = Math.max(robot.DRONE_ANGLE_UP - 0.001, 0);
                positionMsg.setValue( " %6.3f", robot.DRONE_ANGLE_UP);
                telemetry.update();
                while (gamepad1.right_bumper) sleep(100);
            }
        }
    }
}
