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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import common.Drive;
import common.Robot;


/*
 * This file contains an a test of the Road Runner library
 */

@TeleOp(name="RoadRunner Test", group="Test")

public class RoadRunnerTest extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = null;
    private Drive drive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

        drive = new Drive(hardwareMap);

        //while (! robot.vision.cameraReady())
       //     sleep(100);
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(24)
                .strafeLeft(12)
                .addTemporalMarker(3,() -> robot.dropPurplePixel())
                .waitSeconds(1)
                .forward(21)
                .turn(Math.toRadians(-90))
                .forward(50)
                .strafeRight(30)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(24)
                .turn(Math.toRadians(-90))
                .forward(12)
                .addTemporalMarker(3,() -> robot.dropPurplePixel())
                .waitSeconds(1)
                .back(12)
                .strafeLeft(21)
                .forward(50)
                .strafeRight(30)
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(27)
                .addTemporalMarker(2,() -> robot.dropPurplePixel())
                .waitSeconds(1)
                .forward(18)
                .turn(Math.toRadians(-90))
                .forward(50)
                .strafeRight(30)
                .build();
        //drive.followTrajectorySequence(center);

        robot.resetEncoders();
        drive.forward(31.25);

        sleep(500);
        robot.dropPurplePixel();
        drive.forward(23.5);

    }

}
