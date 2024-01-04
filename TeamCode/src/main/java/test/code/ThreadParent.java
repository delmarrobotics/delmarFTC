/*
 * Test code for threads
 */
package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import common.Drive;
import common.Logger;

@TeleOp(name="Thread Test", group="Test")
@SuppressWarnings("unused")
public class ThreadParent extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();
  //Logger logger = Logger.getLogger("MyLog");

  @Override
  public void runOpMode() {
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      telemetry.addData("Status", "Press start");
      telemetry.update();
      Servo servo;


      Drive drive = new Drive(this);
      drive.start();

      waitForStart();

      boolean found;

      //new Thread(child).start();
      runtime.reset();

      while (opModeIsActive()) {

        if (gamepad1.a)
            while (gamepad1.a) sleep(100);

        if (gamepad1.x)
             found = drive.moveToObject(3,0.25, 3000);
      }
  }
}
