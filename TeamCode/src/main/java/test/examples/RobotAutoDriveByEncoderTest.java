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

package test.examples;

import android.icu.util.DateInterval;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

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

@Autonomous(name="Auto Drive By Encoder", group="Test")

public class RobotAutoDriveByEncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel


    private final ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;              // HD Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20 ;              // Gearing  // ToDo verify gearing
    static final double     WHEEL_DIAMETER_INCHES   = (96 / 25.4) ;     // 96 mm while converted to inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     TURN_SPEED              = 0.5;
    static final double     MAX_SPEED               = 0.5 ;
    static final double     MIN_SPEED               = .1;

    static final int        RAMP_COUNT              = 4;
    static final double     RAMP_DISTANCE           = WHEEL_DIAMETER_INCHES * 2 * COUNTS_PER_INCH; // Speed ramp up in encoder counts

    // To increase speed in constant increments of time, use a triangle sequence to calculate the encoder values at which each
    // speed ramp occurs
    static final double     RAMP_INTERVAL           = COUNTS_PER_INCH * 2;  // encoder count of the first speed ramp up.
    double rampIntervals[] = { RAMP_INTERVAL, RAMP_INTERVAL * 3, RAMP_INTERVAL * 6, RAMP_INTERVAL * 10, RAMP_INTERVAL * 15 };
    double speeds[] = { 0.1, 0.2, 0.3, 0.4, 0.5 };
    int breaking[] = { 0, 0, 0, 0, 500};
    int currentSpeed = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
          // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(MAX_SPEED,  72,  72, 10.0);  // S1: Forward 6 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(MAX_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int leftStart;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            leftStart = leftFrontDrive.getCurrentPosition();

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // reset the timeout time and start motion.
            runtime.reset();

            //leftFrontDrive.setPower(Math.abs(speed));
            //rightFrontDrive.setPower(Math.abs(speed));
            //leftBackDrive.setPower(Math.abs(speed));
            //rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()) {

                double rampPower = rampSpeed3(leftFrontDrive.getCurrentPosition(), leftStart, newLeftTarget, speed);

                leftFrontDrive.setPower(rampPower);
                rightFrontDrive.setPower(rampPower);
                leftBackDrive.setPower(rampPower);
                rightBackDrive.setPower(rampPower);

                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();

                /*
                Logger.message("power: %4.2f %4.2f %4.2f %4.2f %4.2f     position: %6d %6d %6d %6d     busy: %b  %b  %b  %b",
                        rampPower,
                        leftFrontDrive.getPower(),
                        rightFrontDrive.getPower(),
                        leftBackDrive.getPower(),
                        rightBackDrive.getPower(),
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition(),
                        leftFrontDrive.isBusy(),
                        rightFrontDrive.isBusy(),
                        leftBackDrive.isBusy(),
                        rightBackDrive.isBusy()
                        );
                 */

                if (rampPower == 0) break;
                if (! leftFrontDrive.isBusy()) break;
                if (! rightFrontDrive.isBusy()) break;

                if (runtime.seconds() >= timeoutS){
                    Logger.message("encoderDrive timed out");
                    break;
                }

                // Display it for the driver.
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            sleep(2000);

            Logger.message("Target / Position  left: %6d %6d  right: %6d %6d",
                    newLeftTarget,
                    leftFrontDrive.getCurrentPosition(),
                    newRightTarget,
                    rightFrontDrive.getCurrentPosition());

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    /**
     * Ramp to motor speed up or down based on the current position and the distance
     * to travel.
     *
     * @param current  current encoder position
     * @param start    start position of the encoder
     * @param end      ending position of the encoder
     * @param speed    maximum speed
     *
     * @return         motor power (0 to speed)
     */
    private double rampSpeed(double current, double start,  double end, double speed) {

        double power1 = ((current - start) / RAMP_DISTANCE) * (speed - MIN_SPEED) + MIN_SPEED;
        double power2 = ((end - current) / RAMP_DISTANCE) * (speed - MIN_SPEED) + MIN_SPEED;

        double power =  Math.min(Math.min(power1, power2), speed);

        Logger.message("power %f %f %f", power1, power2, power);

        return power;
    }

    /**
     * Ramp to motor speed up or down based on the current position and the distance
     * to travel.
     *
     * @param current  current encoder position
     * @param start    start position of the encoder
     * @param end      ending position of the encoder
     * @param speed    maximum speed
     *
     * @return         motor power (0 to speed)
     */
    private double rampSpeed2(double current, double start,  double end, double speed) {

        double power1 = ((current - start) / RAMP_DISTANCE) * (speed - MIN_SPEED) + MIN_SPEED;
        double power2 = ((end - current) / RAMP_DISTANCE) * (speed - MIN_SPEED) + MIN_SPEED;
        double power =  Math.min(power1, power2);

        double interval = (speed - MIN_SPEED) / 4;

        power = power - (power % interval);
        power = Math.min(power, speed);

        Logger.message("power %f %f %f %f", power1, power2, (Math.min(power1, power2) % interval), power);

        return power;
    }

    /**
     * Ramp to motor speed up or down based on the current position and the distance
     * to travel.
     *
     * @param current  current encoder position
     * @param start    start position of the encoder
     * @param end      ending position of the encoder
     * @param speed    maximum speed
     *
     * @return         motor power (0 to speed)
     */
    private double rampSpeed3(double current, double start,  double end, double speed) {
        double power;

        if (current - start >= rampIntervals[currentSpeed] ) {
            if (currentSpeed < rampIntervals.length - 1) {
                currentSpeed++;
            }
        }

        if ((end - current) >  breaking[currentSpeed]){
            power = speeds[currentSpeed];
        } else {
            power = 0;
        }

        Logger.message("power %f %f %f %d %f", current, start, end, currentSpeed, power);

        return power;
    }
}















