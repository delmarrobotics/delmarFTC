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

    public enum GamepadMode { PIXEL, HANGING };
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

            if (! gamepad1.atRest())
            {
                // POV Mode uses left stick to go forward, and right stick to turn.
                double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                double turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                robot.moveRobot(drive, strafe, turn);
            }

            if (robot.hangingArm.control()) {
                continue;
            }

            if (gamepad1.right_trigger > 0) {
                robot.launchDrone();
            }

            if (gamepad1.left_bumper) {

            } else if (gamepad1.right_bumper) {
                // robot lifter controls
                if (gamepad1.left_trigger > 0){
                    while (gamepad1.left_trigger > 0){
                        robot.lifterUp();
                    }
                    robot.lifterStop();
                } else if (gamepad1.right_trigger > 0) {
                    while (gamepad1.right_trigger > 0){
                        robot.lifterDown();
                    }
                    robot.lifterStop();
                } else if (gamepad1.a) {
                    robot.dropPixel();
                }
            } else {
                if (gamepad1.a){
                    //  arm elbow down

                } else if (gamepad1.b) {
                    // grabbing pixel from intake
                } else if (gamepad1.y){
                    // pixel hand opening and closing(2 servos)
                } else if (gamepad1.x){
                    // unused
                }

            }

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
