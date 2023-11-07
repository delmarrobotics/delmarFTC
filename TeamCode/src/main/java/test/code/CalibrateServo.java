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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Config;
import common.Logger;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Calibrate Servo", group="Test")
@SuppressWarnings("test/unused")

public class CalibrateServo extends LinearOpMode {

    /* Declare OpMode members. */
    private final ElapsedTime     runtime = new ElapsedTime();

    private Servo servo   = null;
    private double position = 0;
    private double target = 0;
    private int lastCount = 0;

    private Telemetry.Item startMsg;
    private Telemetry.Item positionMsg;
    private Telemetry.Item directionMsg;
    private Telemetry.Item targetMsg;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press start");
        telemetry.update();

        servo = hardwareMap.get(Servo.class, Config.HAND_LOWER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.setAutoClear(false);
        startMsg = telemetry.addData("Encoder Calibration\nControls", "\n" +
                "  left stick - manual servo control\n" +
                "  left trigger - decrease encoder count\n" +
                "  right trigger - increase encoder count\n" +
                "  a - run motor for encoder count\n" +
                "  x - zero encode count\n" +
                "  b - set encoder count \n\n");

        positionMsg = telemetry.addData("Servo position", 0);
        directionMsg = telemetry.addData("Servo direction", 0);
        targetMsg = telemetry.addData("Target position", 0);

        positionMsg.setValue( "%.0f", servo.getPosition());
        directionMsg.setValue("%s", servo.getDirection());
        targetMsg.setValue("%f", target);

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // Run servo to an target position
                servo.setPosition(target);
                //motor.setDirection(DcMotorSimple.Direction.FORWARD);

            } else if (gamepad1.x) {
                position = 0;
                servo.setPosition(position);
                // zero calibration count


            } else if (gamepad1.b) {
                // set target position
                target = servo.getPosition();

            } else if (gamepad1.right_trigger > 0) {
                // increase calibration count
                runtime.reset();
                while (gamepad1.right_trigger > 0) {
                    target += increment(1, 2, 4);
                    targetMsg.setValue("%f", target);
                    telemetry.update();
                }

            } else if (gamepad1.left_trigger > 0) {
                // decrease calibration count
                runtime.reset();
                while (gamepad1.right_trigger > 0) {
                    target -= increment(1, 2, 4);
                    targetMsg.setValue("%f", target);
                    telemetry.update();
                }

            } else if (gamepad1.left_stick_y > 0) {
                // manually run the servo
                //motor.setDirection(DcMotor.Direction.REVERSE);
                while (true) {
                    //Logger.message("y stick %4.2f", gamepad1.left_stick_y );
                    if (gamepad1.left_stick_y <= 0)  break;
                    position = servo.getPosition() + .01 ;
                    servo.setPosition(position);
                    positionMsg.setValue( "%f", position);
                    sleep(200);
                }

            } else if (gamepad1.left_stick_y < 0) {
                // manually run the motor forward
                //motor.setDirection(DcMotor.Direction.FORWARD);
                while (true) {
                    if (gamepad1.left_stick_y >= 0) break;
                    position = servo.getPosition() - .01;
                    servo.setPosition(position);
                    positionMsg.setValue( "%f", position);
                    sleep(200);
                }

            } else if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up) {
                    servo.setPosition(target);
                }
            }

            positionMsg.setValue( "%f", servo.getPosition());
            telemetry.update();
        }
    }



    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    public int increment(int v1, int v2, int v3){
        int sleepTime;
        int delta;
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
}

