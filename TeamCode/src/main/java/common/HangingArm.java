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
    private static final double ELBOW_HOME_POSITION = 0.97;
    private static final double ELBOW_TARGET_POSITION = 0.64;
    private static final double ELBOW_LOCK_POSITION = 0.69;
    private static final double WRIST_HOME_POSITION = 0.70;
    private static final double WRIST_TARGET_POSITION = 0.96;
    private static final double THUMB_OPEN = 0.5;
    private static final double THUMB_CLOSE = 1;

    private Servo elbow;
    private Servo wrist;
    private Servo thumb;

    public LinearOpMode     opMode;

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
            thumb.setPosition(THUMB_CLOSE);
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

    public void wristUp() {
        wrist.setPosition(WRIST_HOME_POSITION);
    }

    public void wristDown() {
        wrist.setPosition(WRIST_TARGET_POSITION);
    }

    public void thumbOpen() {
        thumb.setPosition(THUMB_OPEN);
    }

    public void thumbClose() {
        thumb.setPosition(THUMB_CLOSE);
    }

    public void displayControls(){
        opMode.telemetry.addData("Hanging Arm Controls (Gamepad 2)", "\n" +
                "  dpad up - arm up\n" +
                "  dpad down - arm down\n" +
                "  dpad left - rotate up\n" +
                "  dpad right - rotate down\n" +
                "  left stick - manual move the elbow\n" +
                "  right stick - manual rotate the hook\n" +
                "  a - release hook\n" +
                "  b - close release\n" +
                "  x - lock in the hook\n" +
                "  y - lift the robot off the ground" +
                "\n");
    }

    /**
     * Manually control the handing arm
     */
    public boolean control() {

        Gamepad gamepad = opMode.gamepad2;
        boolean handled = true;

        if (gamepad.dpad_up) {
            // Raise the hanging arm from its stored position
            elbowUp();
            Logger.message("Hanging Arm Up");

        } else if (gamepad.dpad_down) {
            // Lower the hanging arm to its stored position
            elbowDown();
            Logger.message("Hanging Arm Down");

        } else if (gamepad.dpad_left) {
            // Turn the hook to it hanging position
            wristUp();
            Logger.message("Hanging Wrist up");
        }
        else if (gamepad.dpad_right) {
            // Move the hook to its stored position
            wristDown();
            Logger.message("Hanging Wrist In");

        } else if (gamepad.a) {
            // Release to hook
            thumbOpen();
            Logger.message("Hanging Arm open hook release");

        } else if (gamepad.b) {
            // Close the hook device
            thumbClose();
            Logger.message("Hanging Arm close hook release");

        } else if (gamepad.x) {
            // Move the hook to its lock position
            elbow.setPosition(ELBOW_LOCK_POSITION);

        } else if (gamepad.left_stick_y > 0) {
            // manually move the elbow
            while (gamepad.left_stick_y > 0) {
                double position = elbow.getPosition() + .01 ;
                elbow.setPosition(position);
                Logger.message("elbow position %f", position);
                opMode.sleep(100);
            }

        }  else if (gamepad.left_stick_y < 0) {
            // manually move the elbow
            while (gamepad.left_stick_y < 0) {
                double position = elbow.getPosition() - .01;
                elbow.setPosition(position);
                Logger.message("elbow position %f", position);
                opMode.sleep(100);
            }

        } else if (gamepad.right_stick_y > 0) {
            // manually move the wrist
            while (gamepad.right_stick_y > 0) {
                double position = wrist.getPosition() + .005 ;
                wrist.setPosition(position);
                Logger.message("wrist position %f", position);
                opMode.sleep(100);
            }

        } else if (gamepad.right_stick_y < 0) {
            // manually move the wrist
            while (gamepad.right_stick_y < 0) {
                double position = wrist.getPosition() - .005;
                wrist.setPosition(position);
                Logger.message("wrist position %f", position);
                opMode.sleep(100);
            }

        } else if (gamepad.y) {
            // Lift the robot off the ground
            Logger.message("Hanging Arm lift");
        }

        else {
            handled = false;
        }
        return handled;
    }
}
























