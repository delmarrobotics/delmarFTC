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

package test.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

@TeleOp(name="Pixel Arm Tester", group="Test")
@Disabled
@SuppressWarnings("unused")

public class CalibratePixelArm extends LinearOpMode {

    /* Declare OpMode members. */
    private final ElapsedTime     runtime = new ElapsedTime();

    private DcMotor ladder = null;
    private DcMotor turn = null;
    private int encoderCount = 0;
    private int lastCount = 0;
    private double speed = 0.2;

    private Telemetry.Item startMsg;
    private Telemetry.Item positionMsg;
    private Telemetry.Item speedMsg;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;      // eg: HD Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;      // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = (96 / 25.4);     // Circumference of 96 mm wheel in inches

    static boolean          pixelArmExtended        = false;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press start");
        telemetry.update();

        initMotor();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.setAutoClear(false);
        startMsg = telemetry.addData(" Pixel Arm Tool \nControls", "\n" +
                "  Y - raise arm\n" +
                "  B - extend arm\n" +
                "  X - retract arm\n" +
                "  A - lower arm");

        speedMsg = telemetry.addData("Speed", speed);

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                motorRunToPosition(speed, 0, 15.0);  // 15 second timeout
            } else if (gamepad1.b) {
                motorRunToPosition(speed, 2982, 15.0);  // 15 second timeout was 2982
            } else if (gamepad1.y) {
                turnRunToPosition(speed, -2974, 15);  // 15 second timeout
            } else if (gamepad1.a) {
                turnRunToPosition(speed, 0, 15);  // 15 second timeout
            } else if (gamepad1.left_bumper) {
                speed -= 0.1;
            } else if (gamepad1.right_bumper) {
                speed += 0.1;
            }


            //positionMsg.setValue("%7d", motor.getCurrentPosition());
            telemetry.update();
        }
    }


    /**
     * Initial the motor to calibrate.
     */
    public void initMotor (){
        ladder  = hardwareMap.get(DcMotor.class, "pixelLadder");
        turn = hardwareMap.get(DcMotor.class, "pixelTurn");
        //2982 in extended and 0 at default
        ladder.setDirection(DcMotor.Direction.REVERSE);
        ladder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ladder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ladder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turn.setDirection(DcMotor.Direction.REVERSE);
        turn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     *  Run the motor using the encoder to the specified position.
     *
     * @param speed motor power (0 - 1)
     * @param newPosition encoder counts to move the motor
     * @param timeoutS seconds to run before timing out
     */
    public void motorRunToPosition(double speed, int newPosition, double timeoutS) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            ladder.setTargetPosition(newPosition);
            ladder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ladder.setPower(Math.abs(speed));

            runtime.reset();
            while (opModeIsActive() && ladder.isBusy()) {
                if (runtime.seconds() >= timeoutS) {
                    break;
                }
            }

            if (newPosition > 0)
                pixelArmExtended = true;
            else {
                pixelArmExtended = false;
                // only stop the motor when the arm is lowered
                ladder.setPower(0);
            }
        }
    }


    public void pixelArmAdjustPosition(){
        if (pixelArmExtended) {
            if (ladder.isBusy()) return;
            ladder.setTargetPosition(ladder.getTargetPosition());
        }
    }


    public void turnRunToPosition(double speed, int count, double timeoutS) {
        int newPosition;
        int currentPosition;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newPosition = count;
            turn.setTargetPosition(newPosition);
            turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turn.setPower(Math.abs(speed));

            runtime.reset();
            while (turn.isBusy()) {
                if (! opModeIsActive()) {
                    break;
                }
                if (runtime.seconds() >= timeoutS) {
                    break;
                }
            }

            // Stop all motion;
            turn.setPower(0);
        }
    }
}
