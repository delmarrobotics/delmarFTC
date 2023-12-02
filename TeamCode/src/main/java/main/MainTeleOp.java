package main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Robot;

/*
 * This file contains the OpMode for phase two of the FTS Center Stage Competition.
 *
 */

@TeleOp(name="Main TeleOp", group="Main")
@SuppressWarnings("unused")
public class MainTeleOp extends LinearOpMode {

    public enum GamepadMode { PIXEL, HANGING }
    GamepadMode mode;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize the robot hardware.
    private Robot robot = null;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press start");
        telemetry.update();

        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        displayControls();
        robot.pixelArm.displayControls();
        mode = GamepadMode.PIXEL;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to rotate, left trigger accelerate.
            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            double yaw   = -gamepad1.right_stick_x / 3.0;  // Reduce rotate rate to 33%.
            double speed = gamepad1.left_trigger;
            robot.moveRobot(drive, strafe, yaw, speed);

            // If the start button is pressed, ignore all controls.
            if (gamepad1.start || gamepad2.start) {
                continue;
            }

            if (gamepad1.back) {
                changeGamepadMode();
                while (gamepad1.back) sleep(100);
            }

            if (mode == GamepadMode.HANGING) {
                if (robot.hangingArm.control()) {
                    continue;
                } else if (gamepad2.a) {
                    robot.hangRobot();
                    while (gamepad2.a) sleep(250);
                }

            } else if (mode == GamepadMode.PIXEL) {
                if (robot.pixelArm.control())
                    continue;
            }

            if (gamepad1.left_bumper) {
                if (gamepad1.left_trigger > 0) {
                    // Launch the drone
                    robot.launchDrone();
                    while (gamepad1.left_trigger  > 0) {
                        sleep(100);
                    }
                }

            } else {
                if (gamepad1.left_trigger != 0 ){
                    robot.pixelArm.autoDrop();
                }

                if (gamepad1.a) {
                    // Toggle the intake and the two spinners on / off
                    robot.toggleIntake();
                    while (gamepad1.a) sleep(100);

                } else if (gamepad1.y) {
                    // Reverse the intake and the spinners
                    robot.intakeReverse();
                    while (gamepad1.y) sleep(100);

                } else if (gamepad1.b) {
                    // Raise or lower the pixel intake
                    robot.toggleIntakeRotate();
                    while (gamepad1.b) sleep(100);

                } else if (gamepad1.x) {
                    while (gamepad1.x) sleep(100);
                }
            }

            telemetry.update();
        }
    }

    private void changeGamepadMode () {
        if (mode == GamepadMode.PIXEL) {
            mode = GamepadMode.HANGING;
            displayControls();
            robot.hangingArm.displayControls();
        } else if (mode == GamepadMode.HANGING) {
            mode = GamepadMode.PIXEL;
            displayControls();
            robot.pixelArm.displayControls();
        }
    }

    public void displayControls(){
        telemetry.addLine("Driver Controls (Gamepad 1)\n" +
                "  left stick - drive robot\n" +
                "  right stick - rotate robot\n" +
                "  left trigger - accelerate robot speed\n" +
                "  back - toggle gamepad2 (pixelArm / hangingArm)\n" +
                "  a - intake on / off\n" +
                "  b - intake rotate up / down\n" +
                "  y - intake reverse\n" +
                "\n");
    }

}
