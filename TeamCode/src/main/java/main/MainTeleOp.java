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

        mode = GamepadMode.PIXEL;
        robot.pixelArm.displayControls();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to rotate, left trigger accelerate.
            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            double yaw   = -gamepad1.right_stick_x / 3.0;  // Reduce rotate rate to 33%.
            double speed = gamepad1.left_trigger;
            robot.moveRobot(drive, strafe, yaw, speed);
            //robot.moveRobot(drive, strafe, yaw);

            // If the start button is pressed, ignore all controls.
            if (gamepad1.start || gamepad2.start) {
                continue;
            }

            if (mode == GamepadMode.HANGING) {
                if (robot.hangingArm.control()) {
                    continue;
                }
            } else if (mode == GamepadMode.PIXEL) {
                if (robot.pixelArm.control()) {
                    continue;
                }
            }

            if (gamepad1.back) {
                changeGamepadMode();
                while (gamepad1.back) sleep(100);
            }

            if (gamepad1.left_bumper) {
                if (gamepad1.left_trigger > 0) {
                    // Launch the drone
                    robot.launchDrone();
                    while (gamepad1.left_trigger  > 0) {
                        sleep(100);
                    }

                } else if (gamepad1.a) {
                    hangRobot();

                } else if (gamepad1.dpad_up){
                    while (gamepad1.dpad_up){
                        robot.lifterUp();
                    }
                    robot.lifterStop();

                } else if (gamepad1.dpad_down) {
                    while (gamepad1.dpad_down) {
                        robot.lifterDown();
                    }
                    robot.lifterStop();
                }

            } else if (gamepad1.right_bumper) {
                // robot lifter controls
                if (gamepad1.a) {
                    robot.dropPixel();
                }

            } else {
                if (gamepad1.a) {
                    // Toggle the intake and the two spinners on / off
                    robot.toggleIntake();
                    while (gamepad1.a) sleep(100);
                } else if (gamepad1.b) {
                    // Raise or lower the pixel intake
                    robot.toggleIntakeRotate();
                    while (gamepad1.b) sleep(100);
                } else if (gamepad1.y){
                    while (gamepad1.y) sleep(100);
                } else if (gamepad1.x){
                    while (gamepad1.x) sleep(100);
                }
            }

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    private void hangRobot() {
        robot.hangingArm.wristUp();
        sleep(2000);
        robot.hangingArm.lockInHook();
        robot.moveRobot(.1, 0, 0);
        sleep(500);
        robot.stopRobot();
        robot.hangingArm.thumbOpen();
        robot.hangingArm.elbowRelease();
        sleep(500);
        robot.hangingArm.thumbClose();
        robot.hangingArm.wristDown();
        robot.hangingArm.elbowDown();
        while (gamepad1.a) {
            sleep(250);
        }
    }

    private void changeGamepadMode () {
        if (mode == GamepadMode.PIXEL) {
            mode = GamepadMode.HANGING;
            robot.hangingArm.displayControls();
        } else if (mode == GamepadMode.HANGING) {
            mode = GamepadMode.PIXEL;
            robot.pixelArm.displayControls();
        }
    }
}
