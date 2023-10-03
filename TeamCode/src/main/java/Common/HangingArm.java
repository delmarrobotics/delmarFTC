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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Hardware;

/*
 * This file contains the code to control the the arm of the robot that lifts the robot off
 * the ground
 */

public class HangingArm
{
    static final double     HANGING_ARM_COUNTS_PER_MOTOR_REV    = 384.5 ;  //  GoBilda 5202 435 RPM Motor Encoder


    public OpMode parent;
    private final Hardware robot;

    private final ElapsedTime runtime = new ElapsedTime();

    //constructor
    public HangingArm(OpMode parent, Hardware robot) {
        this.parent = parent;
        this.robot = robot;
    }

    public void init(){

        // Tell the driver that initialization is complete.
        parent.telemetry.addData("Status", "Hanging Arm Initialized");

        calibrateArm(.2, 5, 3000);
    }


    /**
     * Calibrate the telescoping part of the arm
     *
     *  @param speed        Motor power (-1.0 to 1.0) >0 is up
     *  @param revolutions  Number of motor revolutions
     *  @param timeout      Maximum time to run in millisecond
     */
    public void calibrateArm(double speed, int revolutions, double timeout) {

        int targetPosition;
        DcMotor hangingMotor = robot.hangingMotor;

        robot.hangingMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        targetPosition = (int)(revolutions * HANGING_ARM_COUNTS_PER_MOTOR_REV);
        robot.hangingMotor.setTargetPosition(targetPosition);

        robot.hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion.
        runtime.reset();
        robot.hangingMotor.setPower(Math.abs(speed));

        // Keep looping while there is time left, and the motors are running.
        //while (robot.hangingMotor.isBusy() ) {
        while (true ) {
            // Check if timed out
            if (runtime.milliseconds() >= timeout){
                parent.telemetry.addData("Stopped", "Timed out after %6.0f millisecond", timeout);
                parent.telemetry.addData("Currently at",  " at %7d", robot.hangingMotor.getCurrentPosition());
                break;
            }

            // Display it for the driver.
            parent.telemetry.addData("Running to",  " %7d", targetPosition);
            parent.telemetry.addData("Currently at",  " at %7d", robot.hangingMotor.getCurrentPosition());
            parent.telemetry.update();
        }

        // Stop the motion if we timed out
        robot.hangingMotor.setPower(0);
        robot.hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Manually control the handing arm
     */
    public void manualControl() {

        Gamepad gamepad = parent.gamepad1;

        // Raise the hanging arm from its stored position
        if (gamepad.dpad_right) {
            parent.telemetry.addData("Status", "Hanging Arm Out");
            robot.hangingArmOut();
            while (gamepad.dpad_right){ idle(); }
            robot.handingArmStop();
        }
        // Lower the hanging arm to its stored position
        else if (gamepad.dpad_left) {
            parent.telemetry.addData("Status", "Hanging Arm IN");
            robot.hangingArmIn();
            while (gamepad.dpad_left){ idle(); }
            robot.handingArmStop();
        }
        else if (gamepad.dpad_up) {
            parent.telemetry.addData("Status", "Hanging Arm Up");
            robot.hangingArmUp();
            while (gamepad.dpad_up){ idle(); }
            robot.handingArmStop();
        }
        else if (gamepad.dpad_down) {
            robot.hangingArmDown();
            parent.telemetry.addData("Status", "Hanging Arm Down");
            while (gamepad.dpad_down){ idle(); }
            robot.handingArmStop();
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
























