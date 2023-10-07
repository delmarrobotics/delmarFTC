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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains the code to control the the arm of the robot that lifts the robot off
 * the ground
 */
public class HangingArm
{
    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double     HANGING_SERVO_POWER  = 1;    // speed to run the servo the deploys the arm
    static final double     HANDING_MOTOR_POWER  = 1;    // speed to run the motor that extends to arm
    static final double     HANGING_ARM_COUNTS_PER_MOTOR_REV    = 384.5 ;  //  GoBilda 5202 435 RPM Motor Encoder

    public LinearOpMode     opMode;
    public CRServo          hangingServo = null;
    public DcMotor          hangingMotor = null;
    public TouchSensor      hangingTouchSensor = null;

    private final ElapsedTime runtime = new ElapsedTime();

    //constructor
    public HangingArm(LinearOpMode opMode) {
        this.opMode = opMode;
    }
/* ToDo add back in

    public HangingArm(LinearOpMode opMode){
        this.opMode = opMode;
    }
*/

    /**
     * Initialize the hanging arm hardware
     */
    public void init(){

        HardwareMap hardwareMap = opMode.hardwareMap;

        try {
            // Initial the hanging arm
            hangingServo = hardwareMap.get(CRServo.class, "hangingArmServo");
            hangingMotor = hardwareMap.get(DcMotor.class, "hangingArmMotor");
            hangingTouchSensor = hardwareMap.get(TouchSensor.class, "hangingArmUp");
        } catch (Exception e){
            Logger.error(e, "Hanging hardware not found");
        }

        // Tell the driver that initialization is complete.
        opMode.telemetry.addData("Status", "Hanging Arm Initialized");
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

        hangingMotor.setDirection(DcMotor.Direction.FORWARD);
        hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        targetPosition = (int)(revolutions * HANGING_ARM_COUNTS_PER_MOTOR_REV);
        hangingMotor.setTargetPosition(targetPosition);

        hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion.
        runtime.reset();
        hangingMotor.setPower(Math.abs(speed));

        // Keep looping while there is time left, and the motors are running.
        //while (hangingMotor.isBusy() ) {
        while (true ) {
            // Check if timed out
            if (runtime.milliseconds() >= timeout){
                opMode.telemetry.addData("Stopped", "Timed out after %6.0f millisecond", timeout);
                opMode.telemetry.addData("Currently at",  " at %7d", hangingMotor.getCurrentPosition());
                break;
            }

            // Display it for the driver.
            opMode.telemetry.addData("Running to",  " %7d", targetPosition);
            opMode.telemetry.addData("Currently at",  " at %7d", hangingMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop the motion if we timed outls
        hangingMotor.setPower(0);
        hangingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Manually control the handing arm
     */
    public void manualControl() {

        Gamepad gamepad = opMode.gamepad1;

        // Raise the hanging arm from its stored position
        if (gamepad.dpad_right) {
            Logger.message("Hanging Arm Out");
            hangingServo.setDirection(DcMotor.Direction.FORWARD);
            hangingServo.setPower(HANGING_SERVO_POWER);
            while (opMode.gamepad1.dpad_right) {
                opMode.telemetry.addData("Status", "Hanging Arm Out");
                opMode.telemetry.update();
            }
            hangingServo.setPower(0);
        }
        // Lower the hanging arm to its stored position
        else if (gamepad.dpad_left) {
            Logger.message("Hanging Arm In");
            hangingServo.setDirection(DcMotor.Direction.REVERSE);
            hangingServo.setPower(HANGING_SERVO_POWER);
            while (gamepad.dpad_left){
                opMode.telemetry.addData("Status", "Hanging Arm IN");
                opMode.telemetry.update();
            }
            hangingServo.setPower(0);
        }
        // Extend the telescoping part the the arm
        else if (gamepad.dpad_up) {
            Logger.message("Hanging Arm Up");
            hangingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hangingMotor.setPower(HANDING_MOTOR_POWER);
            while (gamepad.dpad_up){
                opMode.telemetry.addData("Status", "Hanging Arm Up");
                opMode.telemetry.update();
            }
            hangingMotor.setPower(0);
        }
        // Retract the telescoping part the the arm
        else if (gamepad.dpad_down) {
            Logger.message("Hanging Arm Down");
            hangingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            hangingMotor.setPower(HANDING_MOTOR_POWER);
            while (gamepad.dpad_down){
                opMode.telemetry.addData("Status", "Hanging Arm Down");
                opMode.telemetry.update();
            }
            hangingMotor.setPower(0);
        }
        // Automatically raise the hanging arm to its fully deployed position
        else if (gamepad.y) {
            Logger.message("Hanging Arm Automatically raise");
            armUp();
        }
        // Lift the robot off the ground
        else if (gamepad.a) {
            Logger.message("Hanging Arm lift");
            armLift();
        }
        // Calibrate the telescoping part of the arm
        else if (gamepad.b) {
            Logger.message("Hanging Arm Calibrate");
            calibrateArm(.2, 2, 300);
        }
    }


    /**
     *  Returns true if the hanging arm is in its full upright position
     */
    public Boolean hangingArmIsUp(){
        return hangingTouchSensor.isPressed();
    }


    /**
     * Automatically raise the hanging arm to its fully deployed position
     */
    public void armUp(){

    }

    /**
     * Lift the robot off the ground
     */
    public void armLift()
    {
    }

}
























