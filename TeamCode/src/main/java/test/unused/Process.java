/*
 *
 */
package test.unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Logger;

public class Process implements Runnable {
//public class ThreadChild extends Thread {

    public boolean running = true;

    public Process() {
    }

    public void sleep(long milliseconds){
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

    }
}
