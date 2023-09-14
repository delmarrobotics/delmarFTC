package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="mainCode")
public class mainJava extends OpMode
{

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    float speed;
    float strafe;
    float turn;

    float strength;

    @Override
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
    }

    @Override
    public void loop()
    {
        speed = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        if(gamepad1.a)
        {
            strength = 1f;
        }
        if(gamepad1.b) {
            strength = 0.75f;
        }
        if(gamepad1.x)
        {
            strength = 0.5f;
        }
        if(gamepad1.y) {
            strength = 0.25f;
        }

        /* frontLeft.setPower(speed + strafe - turn);
        frontRight.setPower(speed - strafe - turn);
        backLeft.setPower(speed - strafe + turn);
        backRight.setPower(speed + strafe - turn); */

        frontLeft.setPower((speed - turn - strafe) * strength);
        frontRight.setPower((speed + turn + strafe) * strength);
        backLeft.setPower((0 - speed + turn - strafe) * strength);
        backRight.setPower((0 - speed - turn + strafe) * strength);
    }

}
