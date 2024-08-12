/*
 * The class contain support the the autonomous phase of the Center Stage competition
 */
package common;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Auto extends Thread {

    private final ElapsedTime cameraReadyTime = new ElapsedTime();

    LinearOpMode opMode;
    Robot robot;

    public Auto(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
        this.start();
    }

    public void run() {
        Logger.message("auto thread started");
        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
        }

        Logger.message("auto thread stopped");
    }


    public void waitForCamera() {
        while (opMode.opModeIsActive() && !robot.vision.cameraReady())
            opMode.sleep(200);
        opMode.sleep(4000); //sometimes the camera wont init even when robot.vision.cameraReady()
        cameraReadyTime.reset();
    }
}