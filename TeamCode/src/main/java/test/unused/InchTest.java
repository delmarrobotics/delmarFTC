package test.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import common.Robot;

@TeleOp(name="inchTest", group="Main")
@Disabled
public class InchTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Initialize the robot hardware.
    private Robot robot = null;

    int inchDrive;

    public void runOpMode() {
        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) inchDrive += 1;
            if (gamepad1.b) inchDrive -= 1;

            if (gamepad1.left_bumper) robot.drive.moveDistance(Drive.DIRECTION.FORWARD,0.5,inchDrive,1000);
        }
    }
}
