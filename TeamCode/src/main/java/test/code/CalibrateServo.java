/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.SortedSet;

import common.Config;
import common.Logger;

/*
 * This OpMode calibrate any of the robot servos.
 */

@TeleOp(name="Calibrate Servo", group="Test")
@SuppressWarnings("test/unused")

public class CalibrateServo extends LinearOpMode {

    /* Declare OpMode members. */
    private final ElapsedTime     runtime = new ElapsedTime();

    private Servo servo   = null;
    private double position = 0;
    private double home = .5;
    private double target = 1;

    class ServoInfo {
        String  name;
        Servo   servo;
    }
    ServoInfo[] servos = new ServoInfo[12];
    int servoCount;
    int currentServo;



    @Override
    public void runOpMode() {

        getServos();

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Servo Calibration Controls", "\n" +
                "  dpad up - select next servo\n" +
                "  left stick - manual control servo position\n" +
                "  left trigger - decrease target position\n" +
                "  right trigger - increase target position\n" +
                "  left bumper - decrease home position\n" +
                "  right bumper - increase home position\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  x - run to home position\n" +
                "  b - run servo to target position\n" +
                "\n");

        Telemetry.Item servoNameMsg =  telemetry.addData("Servo name", 0);
//        Telemetry.Item servoPortMsg = telemetry.addData("Servo port", 0);
        Telemetry.Item directionMsg = telemetry.addData("Servo direction", 0);
        Telemetry.Item positionMsg = telemetry.addData("Servo position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        servoNameMsg.setValue("%s", servos[currentServo].name);
//        servoPortMsg.setValue("%d", servos[currentServo].servo.getPortNumber());
        directionMsg.setValue("%s", servo.getDirection());
        positionMsg.setValue( "%f", servo.getPosition());
        homeMsg.setValue("%f", home);
        targetMsg.setValue("%f", target);

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                target = servo.getPosition();
                targetMsg.setValue("%f", target);

            } else if (gamepad1.y) {
                // set the home position to the current position
                home = servo.getPosition();
                homeMsg.setValue("%f", home);

            } else if (gamepad1.x) {
                // run to zero position
                servo.setPosition(home);

            } else if (gamepad1.b) {
                // Run servo to an target position
                servo.setPosition(target);

            } else if (gamepad1.right_trigger > 0) {
                // increase target position
                runtime.reset();
                while (gamepad1.right_trigger > 0) {
                    target += increment(.01, .02, .04);
                    targetMsg.setValue("%f", target);
                    telemetry.update();
                }

            } else if (gamepad1.left_trigger > 0) {
                // decrease the target position
                runtime.reset();
                while (gamepad1.left_trigger > 0) {
                    target -= increment(.01, .02, .04);
                    targetMsg.setValue("%f", target);
                    telemetry.update();
                }

            } else if (gamepad1.right_bumper) {
                // increase target position
                runtime.reset();
                while (gamepad1.right_bumper) {
                    home += increment(.01, .02, .04);
                    homeMsg.setValue("%f", home);
                    telemetry.update();
                }

            } else if (gamepad1.left_bumper) {
                // decrease the target position
                runtime.reset();
                while (gamepad1.left_bumper) {
                    home -= increment(.01, .02, .04);
                    homeMsg.setValue("%f", home);
                    telemetry.update();
                }

            } else if (gamepad1.left_stick_y > 0) {
                // manually run the servo
                while (gamepad1.left_stick_y > 0) {
                    //Logger.message("y stick %4.2f", gamepad1.left_stick_y );
                    position = servo.getPosition() + .01 ;
                    servo.setPosition(position);
                    positionMsg.setValue( "%f", position);
                    telemetry.update();
                    sleep(100);
                }

            } else if (gamepad1.left_stick_y < 0) {
                // manually run the motor forward
                while (gamepad1.left_stick_y < 0) {
                    position = servo.getPosition() - .01;
                    servo.setPosition(position);
                    positionMsg.setValue( "%f", position);
                    telemetry.update();
                    sleep(100);
                }

            } else if (gamepad1.dpad_up) {
                nextServo();
                while (gamepad1.dpad_up) {
                    sleep(10);
                }
                servoNameMsg.setValue("%s", servos[currentServo].name);
//                servoPortMsg.setValue("%d", servos[currentServo].servo.getPortNumber());

            } else if (gamepad1.dpad_down) {
                // change the direction of the servo
                if (servo.getDirection() == Servo.Direction.FORWARD)
                    servo.setDirection(Servo.Direction.REVERSE);
                else
                    servo.setDirection(Servo.Direction.FORWARD);
                directionMsg.setValue("%s", servo.getDirection());
                while (gamepad1.dpad_down) sleep(10);
            }

            positionMsg.setValue( "%f", servo.getPosition());
            telemetry.update();
        }
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    public double increment(double v1, double v2, double v3){
        int sleepTime;
        double delta;
        if (runtime.seconds() < 3){
            delta = v1;
            sleepTime = 500;
        }
        else if (runtime.seconds() < 6){
            delta = v2;
            sleepTime = 200;
        }
        else{
            delta = v3;
            sleepTime = 100;
        }
        sleep(sleepTime);
        return delta;
    }

    /**
     * Build a list of servos sorted by port number
     */
    private void getServos(){

        servoCount = 0;
        SortedSet<String> names = hardwareMap.getAllNames(Servo.class);
        for (String name: names){
            Servo s = hardwareMap.get(Servo.class, name);
            int port = s.getPortNumber();
            servos[port] = new ServoInfo();
            servos[port].name = name;
            servos[port].servo = s;
            servoCount++;
        }

        for (int i = 0; i < servos.length; i++) {
            if (servos[i] != null) {
                if (servo == null) {
                    servo = servos[i].servo;
                    currentServo = i;
                }
                Logger.message("servo name %s port %d", servos[i].name, servos[i].servo.getPortNumber());
            }
        }
    }

    private void nextServo(){

        int index;
        for (int i = currentServo + 1; i <= currentServo + servos.length; i++) {
            index = i %  servos.length;
            if (servos[index] != null){
                servo = servos[index].servo;
                currentServo = index;
                break;
            }
        }
    }

}

