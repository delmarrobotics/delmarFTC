package main;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Robot;

/*
 * This file contains the OpMode for phase two of the FTS Center Stage Competition.
 *
 */

@TeleOp(name="Main TeleOp", group="Main")
public class MainTeleOp extends LinearOpMode {

    public enum GamepadMode { NONE, PIXEL, HANGING };
    GamepadMode mode;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

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

        mode = GamepadMode.HANGING;
        robot.hangingArm.displayControls();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to rotate.
            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            double yaw   = -gamepad1.right_stick_x / 3.0;  // Reduce rotate rate to 33%.
            robot.moveRobot(drive, strafe, yaw);

            if (mode == GamepadMode.HANGING) {
                if (robot.hangingArm.control()) {
                    continue;
                }
            } else if (mode == GamepadMode.PIXEL) {
                if (robot.pixelArm.control()) {
                    continue;
                }
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
                if (gamepad1.a){
                    robot.toggleIntake();
                    while (gamepad1.a) sleep(100);
                } else if (gamepad1.b) {
                    // grabbing pixel from intake
                } else if (gamepad1.y){
                    // pixel hand opening and closing(2 servos)
                } else if (gamepad1.x){

                } else if (gamepad2.a){
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
}
