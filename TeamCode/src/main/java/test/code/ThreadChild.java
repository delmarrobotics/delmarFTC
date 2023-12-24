/*
 *
 */
package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Logger;

public class ThreadChild extends Thread {

    LinearOpMode opMode;
    private final ElapsedTime runtime = new ElapsedTime();
    boolean running = true;

    public ThreadChild(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private void sleepFor(long milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void end() {
        Logger.message("stopping thread");
        Thread.currentThread().interrupted();
        running = false;
    }

    public void run() {
        while (running && opMode.opModeIsActive()) {

            opMode.telemetry.addData("Child", "time: %8.0f", runtime.seconds());
            opMode.telemetry.update();
            sleepFor(500);
        }
    }
}

/*
public class ThreadChild extends Process {

    LinearOpMode opMode;
    private final ElapsedTime runtime = new ElapsedTime();

    public ThreadChild(LinearOpMode opMode) {
        super();
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while (running && opMode.opModeIsActive()) {

            opMode.telemetry.addData("Child", "time: %8.0f", runtime.seconds());
            opMode.telemetry.update();
            sleep(500);
        }
    }
*/
