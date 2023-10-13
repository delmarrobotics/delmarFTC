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

@TeleOp(name="Main TeleOp Two", group="Main")
public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialize the robot hardware.
    private Robot robot = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forw ard, and right stick to turn.
            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            double turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            robot.moveRobot(drive, strafe, turn);

            if (gamepad1.left_bumper) {
                robot.moveByDistance(0.5,12,12,10);
                sleep(250);
            } else if (gamepad1.right_bumper) {

            } else {
                if (gamepad1.a){
                    //  arm retracting
                } else if (gamepad1.b) {
                    // grabbing pixel from intake
                } else if (gamepad1.y){
                    // pixel hand opening and closing(2 servos)
                } else if (gamepad1.x){
                    // unused
                }

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
