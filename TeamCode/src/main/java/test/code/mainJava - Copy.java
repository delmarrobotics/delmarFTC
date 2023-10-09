package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintStream;

@Autonomous(name="mainCode")
public class mainJava extends OpMode
{
    private DistanceSensor Distance=null ;

    ColorSensor color;


    Servo preload;

    float speed;
    float strafe;
    float turn;
    float april_TAG;

    float strength;
    float Change;
    float max;
    float min;
float place;



    @Override
    public void init()
    {
        color = hardwareMap.colorSensor.get("color");
        preload = hardwareMap.servo.get("preload");
    }
    @Override

    public void loop() {

        speed = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) Distance;
        telemetry.addData(">>", "Press start to continue");
        // generic DistanceSensor methods.


        // Rev2mDistanceSensor specific methods.
        telemetry.addData("r", color.red());
        telemetry.addData("b", color.blue());
        telemetry.addData("g", color.green());
        telemetry.addData("a", color.alpha());
        telemetry.update();
        if (color.red() > 400 && color.green() < 1000 && color.blue() < 100) {
            preload.setPosition(0);

        }
        if (color.red() < 400 || color.blue() < 400) {
            preload.setPosition(.90);
        }
        if (gamepad1.a) {
            strength = 1f;

        }
        if (gamepad1.b) {
            strength = 0.75f;
        }
        if (gamepad1.x) {
            strength = 0.5f;
        }
        if (gamepad1.y) {
            strength = 0.25f;
        }


    }}