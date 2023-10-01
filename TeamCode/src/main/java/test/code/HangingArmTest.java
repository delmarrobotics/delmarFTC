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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Test: Hanging Arm", group="Test")

public class HangingArmTest extends OpMode
{
    // Name from robot configuration file on the driver station
    final String HANGING_ARM_SERVO = "hangingArmServo";
    final String HANGING_ARM_MOTOR = "hangingArmMotor";
    final String HANDING_ARM_DEPLOY_UP = "hangingArmDeployUp";

    final double ARM_EXTEND_SPEED = 1;          // speed to run the servo the deploys the arm
    final double ARM_DEPLOY_POWER = 1;          // speed to run the motor that extends to arm

    // Declare OpMode members.
    private ElapsedTime runtime = null;
    private CRServo hangingDeploy = null;
    private DcMotor hangingTelescope = null;
    private TouchSensor deployUp = null;

    //----------------------------------------------
    // * init - Code to run ONCE when the driver hits INIT
    //---------------------------------------------
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        hangingTelescope = hardwareMap.get(DcMotor.class, HANGING_ARM_MOTOR);
        hangingDeploy  = hardwareMap.get(CRServo.class, HANGING_ARM_SERVO);
        deployUp = hardwareMap.get(TouchSensor.class, HANDING_ARM_DEPLOY_UP);

        runtime = new ElapsedTime();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Hanging Arm Initialized");
    }

    //----------------------------------------------
    // start - Code to run ONCE when the driver hits PLAY
    //----------------------------------------------
    @Override
    public void start() {
        runtime.reset();
    }

    //----------------------------------------------
    // loop - Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    //----------------------------------------------
    @Override
    public void loop() {

        telemetry.addData("Status", ": " + deployUp.isPressed());

        if (gamepad1.x || gamepad1.b){
            if (gamepad1.b){
                hangingDeploy.setDirection(DcMotor.Direction.FORWARD);
            } else {
                hangingDeploy.setDirection(DcMotor.Direction.REVERSE);
            }
            hangingDeploy.setPower(ARM_DEPLOY_POWER);
        } else {
            hangingDeploy.setPower(0);
        }

        // Extent or retract the hanging arm
        if (gamepad1.y || gamepad1.a) {
            if (gamepad1.y) {
                hangingTelescope.setDirection(DcMotor.Direction.FORWARD);
            } else {
                hangingTelescope.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            hangingTelescope.setPower(ARM_EXTEND_SPEED);
        } else  {
            hangingTelescope.setPower(0);
        }


        /*
        if (gamepad1.x && ! servoUpOn){
                hangingDeploy.setDirection(DcMotor.Direction.FORWARD);
                hangingDeploy.setPower(1);
                servoUpOn = true;
            } else if (! gamepad1.x && servoUpOn) {
                hangingDeploy.setPower(0);
                servoUpOn = false;
            } else if (gamepad1.b && ! servoDownOn){
                hangingDeploy.setDirection(DcMotor.Direction.REVERSE);
                hangingDeploy.setPower(1);
                servoDownOn = true;
            } else if (! gamepad1.b && servoDownOn) {
                hangingDeploy.setPower(0);
                servoDownOn = false;
            }
        */


        // Show the elapsed game time and arm position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
}
