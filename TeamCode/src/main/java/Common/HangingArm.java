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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import common.Hardware;

/*
 * This file contains the code to control the the arm of the robot that lifts the robot off
 * the ground
 */

public class HangingArm
{
    public OpMode parent;
    private Hardware robot;

    //constructor
    public HangingArm(OpMode parent, Hardware robot) {
        this.parent = parent;
        this.robot = robot;
    }

    public void init(){

        // Tell the driver that initialization is complete.
        parent.telemetry.addData("Status", "Hanging Arm Initialized");
    }

    /**
     * Manually control the handing arm
     */
    public void manualControl() {

        Gamepad gamepad = parent.gamepad1;

        // Raise the hanging arm from its stored position
        if (gamepad.dpad_right) {
            robot.hangingArmOut();
            while (gamepad.dpad_right){
                idle();
            }
            robot.handingArmStop();
            parent.telemetry.addData("Status", "Hanging Arm Out");
        }
        // Lower the hanging arm to its stored position
        else if (gamepad.dpad_left) {
            parent.telemetry.addData("Status", "Hanging Arm IN");
        }
        else if (gamepad.dpad_up) {
            parent.telemetry.addData("Status", "Hanging Arm Up");

        }
        else if (gamepad.dpad_down) {
            parent.telemetry.addData("Status", "Hanging Arm Down");

        }
    }

    /**
     * Automatically raise the hanging arm to its fully deployed position
     */
    public void armUp(){

    }

    /**
     * Lift the robot
     */
    public void armLift()
    {
    }

    public void idle() {
        Thread.yield();
    }
}
























