package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "Test")
public class SensorDistanceTest extends LinearOpMode {

    //add the distance sensor instance
    DistanceSensor distanceSensor;

    public void runOpMode() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();
        telemetry.addLine("REV distance sensor test\n");

        while(opModeIsActive()) {
            telemetry.addData("IN", distanceSensor.getDistance(DistanceUnit.INCH) + " IN");
            telemetry.addData("CM", distanceSensor.getDistance(DistanceUnit.CM) + " CM");
            telemetry.update();
        }
    }
}
