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

package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file contains the code to control the the arm of the robot that lifts the robot off
 * the ground
 */
public class HangingArm
{
    // Servo positions
    public static final double ELBOW_HOME_POSITION = 0.97;
    public static final double ELBOW_TARGET_POSITION = 0.59;
    public static final double ELBOW_RELEASE_POSITION = 0.59;
    public static final double WRIST_HOME_POSITION = 0.30;
    public static final double WRIST_TARGET_POSITION = 0.59;
    public static final double THUMB_OPEN = 0.30;
    public static final double THUMB_CLOSE = 0.68;

    private Servo elbow;                     // Servo that raises and lowers the arm
    private Servo wrist;                     // Servo that rotates the hook
    private Servo thumb;                     // Servo that holds the hook
    private DcMotor lifter;                  // Motor to lift the robot off the ground

    public LinearOpMode opMode;

    //constructor
    public HangingArm(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    /**
     * Initialize the hanging arm hardware
     */
    public void init(){

        HardwareMap hardwareMap = opMode.hardwareMap;

        try {
            // Initial the hanging arm
            elbow = hardwareMap.get(Servo.class, Config.HANGING_ELBOW);
            wrist = hardwareMap.get(Servo.class, Config.HANGING_WRIST);
            thumb = hardwareMap.get(Servo.class, Config.HANDING_THUMB);

            lifter = opMode.hardwareMap.get(DcMotor.class, Config.LIFTING_WENCH);
            lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        } catch (Exception e){
            Logger.error(e, "Hanging arm hardware not found");
        }
    }

    /**
     * Automatically raise the hanging arm to its fully deployed position
     */
    public void elbowUp(){
        elbow.setPosition(ELBOW_TARGET_POSITION);
    }

    public void elbowDown(){
        elbow.setPosition(ELBOW_HOME_POSITION);
    }

    public void elbowRelease() { elbow.setPosition(ELBOW_RELEASE_POSITION); }

    public void wristUp() {
        wrist.setPosition(WRIST_TARGET_POSITION);
    }

    public void wristDown() {
        wrist.setPosition(WRIST_HOME_POSITION);
    }

    public void thumbOpen() {
        thumb.setPosition(THUMB_OPEN);
    }

    public void thumbClose() {
        thumb.setPosition(THUMB_CLOSE);
    }

    public void thumbToggle() {
        if (thumb.getPosition() == THUMB_CLOSE)
            thumb.setPosition(THUMB_OPEN);
        else
            thumb.setPosition(THUMB_CLOSE);
    }

    public void lockInHook () {
        double position;
        for (int i = 0; i < 2; i++) {
            position = elbow.getPosition() - 0.0125;
            elbow.setPosition(position);
            position = wrist.getPosition() + 0.015;
            wrist.setPosition(position);
            opMode.sleep(100);
        }
    }

    public void storeArm () {
        elbow.setPosition(ELBOW_HOME_POSITION - ((ELBOW_HOME_POSITION - ELBOW_TARGET_POSITION) / 2)) ;
        opMode.sleep(1000);
        wristDown();
        thumbClose();
        opMode.sleep(1000);
        elbowDown();
    }

    /**
     * Turn on the motor that drives the lifting wench
     */
    public void lifterUp() {
        lifter.setPower(-1);
    }

    /**
     * Turn on the motor that drives the lifting wench
     */
    public void lifterDown(){
        lifter.setPower(1);
    }

    /**
     * Turn off the motor that drives the lifting wench
     */
    public void lifterStop(){
        lifter.setPower(0);
    }

    public void displayControls(){
        opMode.telemetry.addLine("Hanging Arm Controls (Gamepad 2)\n" +
                "  a - raise the arm\n" +
                "  b - lock in the hook\n" +
                "  x - release hook\n" +
                "  y - lower the arm\n" +
                "  dpad up - lift the robot off the ground\n"
                //"  left stick - manual move the elbow\n" +
                //"  right stick - manual rotate the hook\n"
                //"  right bumper - toggle hook release\n" +
                //"  dpad left - rotate up\n" +
                //"  dpad right - rotate down\n" +
                );
    }

    /**
     * Manually control the handing arm
     */
    public boolean control() {

        Gamepad gamepad = opMode.gamepad2;
        boolean handled = true;

        if (gamepad.a) {
            //Raise the arm
            elbowUp();
            Logger.message("Hanging Arm Up");

        } else if (gamepad.b) {
            // lock in the hook, handled in MainTeleOp
            handled = false;
            Logger.message("Hanging Arm lock in hook");

        } else if (gamepad.x) {
            // Release the hook
            thumbOpen();
            Logger.message("Hanging Arm release hook");

        } else if (gamepad.y) {
            // lower the hanging arm
            //TODO Axel wants to change this so that the elbow goes down before the wrist goes down - test when time allows
            wristDown();
            elbowDown();
            opMode.sleep(1500);
            thumbClose();
            Logger.message("Hanging Arm Down");

        } else  if (gamepad.dpad_up) {
            // Raise the robot off the ground
            while (gamepad.dpad_up) lifterUp();
            lifterStop();
            Logger.message("Lower the robot to the ground");

        } else if (gamepad.dpad_down) {
            // Lower the robot to the ground
            while (gamepad.dpad_down) lifterDown();
            lifterStop();
            Logger.message("Lower the robot to the ground");

        } else if (gamepad.dpad_left) {
            // Move the hook to its stored position
            wristDown();
            Logger.message("Hanging wrist to stored position");

        } else if (gamepad.dpad_right) {
            // Turn the hook to it hanging position
            wristUp();
            Logger.message("Hanging wrist to hook position");

        } else if (gamepad.right_bumper) {
            // toggle the hook release
            thumbToggle();
            while (gamepad.right_bumper) opMode.sleep(100);
            Logger.message("Hanging Arm toggle thumb open / close");

        } else  if (gamepad.left_stick_y != 0) {
            // manually move the elbow
            while (gamepad.left_stick_y != 0) {
                double position = elbow.getPosition();
                if (Double.isNaN(position))
                    position = ELBOW_HOME_POSITION;
                if (gamepad.left_stick_y > 0)
                    position += .01;
                else if (gamepad.left_stick_y < 0)
                    position -= 0.01;
                elbow.setPosition(position);
                opMode.sleep(100);
            }
            Logger.message("elbow position %f", elbow.getPosition());

        }  else if (gamepad.right_stick_y > 0) {
            // manually move the wrist
            while (gamepad.right_stick_y > 0) {
                double position = wrist.getPosition() + .005 ;
                // Check the the position is defined, getPosition previously called.
                if (! Double.isNaN(position)) {
                    wrist.setPosition(position);
                    opMode.sleep(100);
                }
            }
            Logger.message("wrist position %f", wrist.getPosition());

        } else if (gamepad.right_stick_y < 0) {
            // manually move the wrist
            while (gamepad.right_stick_y < 0) {
                double position = wrist.getPosition() - .005;
                // Check the the position is defined, getPosition previously called.
                if (! Double.isNaN(position)) {
                    wrist.setPosition(position);
                    opMode.sleep(100);
                }
            }
            Logger.message("wrist position %f", wrist.getPosition());

        }  else if (gamepad.right_stick_y != 0) {
            // manually move the wrist
            while (gamepad.right_stick_y != 0) {
                double position = wrist.getPosition();
                // Check the the position is defined, getPosition previously called.
                if (Double.isNaN(position))
                    position = WRIST_HOME_POSITION;
                else if (gamepad.right_stick_y > 0)
                    position += 0.005;
                else if (gamepad.right_stick_y < 0)
                    position -= 0.005;
                wrist.setPosition(position);
                opMode.sleep(100);
            }
            Logger.message("wrist position %f", wrist.getPosition());

        } else {
            handled = false;
        }
        return handled;
    }
}
























