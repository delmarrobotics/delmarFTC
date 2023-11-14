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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private final ElapsedTime runtime = new ElapsedTime();

    public LinearOpMode     opMode;

    //constructor
    public HangingArm(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    private class Buttons {
        boolean yPressed = false;
    }
    Buttons buttons;

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

    /**
     * Manually control the handing arm
     */
    public void control() {

        Gamepad gamepad = opMode.gamepad1;

        // Raise the hanging arm from its stored position
        if (gamepad.dpad_right) {
            wristUp();
            Logger.message("Hanging Wrist up");
        }
        // Lower the hanging arm to its stored position
        else if (gamepad.dpad_left) {
            wristDown();
            Logger.message("Hanging Wrist In");
        }
        // Extend the telescoping part the the arm
        else if (gamepad.dpad_up) {
            elbowUp();
            Logger.message("Hanging Arm Up");
        }
        // Retract the telescoping part the the arm
        else if (gamepad.dpad_down) {
            elbowDown();
            Logger.message("Hanging Arm Down");
        }

        else if (gamepad.a) {
            thumbOpen();
            Logger.message("Hanging Arm open hook release");
        }
        else if (gamepad.b) {
            thumbClose();
            Logger.message("Hanging Arm close hook release");
        } else if (gamepad.x) {
            elbow.setPosition(ELBOW_LOCK_POSITION);
        }

        // Open or close the hook release
        else if (gamepad.y) {
            if (thumb.getPosition() == THUMB_CLOSE){
            } else {
            }
        }
        // Lift the robot off the ground
        else if (gamepad.x) {
            Logger.message("Hanging Arm lift");
        }
    }
}
























