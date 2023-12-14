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

import common.Auto;
import common.Drive;
import common.Logger;
import common.PixelArm;
import common.Robot;


/*
 * This file contains an a test of the Road Runner library
 */

@TeleOp(name="RoadRunner Test", group="Test")

public class RoadRunnerTest extends LinearOpMode {

    // Declare OpMode members.
    public Robot robot = null;
    private Drive drive = null;
    Auto auto;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the robot hardware.
        robot = new Robot(this);
        robot.init();

        drive = new Drive(hardwareMap);
        drive.setRobot(robot);

        auto = new Auto(this, robot, drive);

        while (! robot.vision.cameraReady())
            sleep(100);
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        auto.setColor(Auto.COLOR.RED);
        auto.objectPosition = Auto.POSITION.right;
        redRightRight();

        //redRightLeft();
        //redRightCenter();

        //blueRightRight();

        //yellowPixel();
        //turnTest();
        //robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.25, 2000);
    }

    public void redRightLeft() {
        robot.forward(24);
        robot.turn(90);
        robot.forward(6);
        robot.dropPurplePixel();
        robot.back(10);
        robot.turn(180);
        robot.forward(33);
        robot.strafeLeft(6);
    }

    public void redRightCenter() {
        auto.setColor(Auto.COLOR.RED);
        robot.forward(30.5);
        robot.dropPurplePixel();
        robot.back(8);
        robot.turn(-90);
        robot.forward(20);
        robot.strafeLeft(3);
        robot.forward(17);
        sleep(10000);
        robot.back(2);
        robot.strafeRight(20);
        robot.forward(12);

        /*
        robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_UP_LOW);
        robot.pixelArm.pixelArmMove(PixelArm.PIXEL_ARM_OUT_LOW);
        robot.pixelArm.pixelWristMove(PixelArm.PIXEL_WRIST_DROP_LOW);
        robot.pixelArm.pixelWristMove(PixelArm.PIXEL_WRIST_HOME);
        robot.pixelArm.pixelArmMove(PixelArm.PIXEL_ARM_IN);
        robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_DOWN);
        sleep(5000);
         */
    }

    public void redRightRight(){

        auto.setColor(Auto.COLOR.RED);
        robot.forward(30);
        robot.turn(-90);
        robot.forward(6);
        robot.dropPurplePixel();
        robot.forward(15);
        robot.strafeRight(7);
        robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.25, 2000);
        auto.yellowPixel();
        robot.forward(7);
    }


    public void blueRightRight(){


        /*
        robot.forward(6.5);
        robot.turn(95);
        robot.forward(24);
        robot.turn(95);
        robot.forward(90);
        double heading = robot.getOrientation();
        Logger.message("forward heading %6.2f", heading);
        robot.strafeLeft(40);
        auto.yellowPixel();

         */
    }




    public void yellowPixel(){
        auto.setColor(Auto.COLOR.RED);
        auto.yellowPixel();

    }




    public void turnTest () {
        for (int i=0; i < 12; i++) {
            robot.turn(-90);
            sleep(1000);
        }
    }

    public void distanceTest () {
        double distance = 24;
        for (distance = 12; distance <= 36; distance += 12) {
            drive.forward(distance);
            drive.back(distance);
//            drive.turn(Math.toRadians(90));
  //          drive.strafeLeft(distance);
    //        drive.strafeRight(distance);
        }
    }
}
