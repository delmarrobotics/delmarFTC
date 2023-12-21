package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import common.Drive;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Thread Test", group="Test")
@SuppressWarnings("unused")
public class ThreadParent extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();
  //Logger logger = Logger.getLogger("MyLog");
  ThreadChild child  = null;

  @Override
  public void runOpMode() {
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      telemetry.addData("Status", "Press start");
      telemetry.update();
      Servo servo;

      //ThreadChild child = new ThreadChild(this);
      //child.start();

      Drive drive = new Drive(this);
      drive.start();

      waitForStart();


      //new Thread(child).start();
      runtime.reset();

      while (opModeIsActive()) {

        if (gamepad1.a)
            servo = hardwareMap.get(Servo.class, "zzz");
        if (gamepad1.x)
          child.end();

        telemetry.addData("Parent", "time: %8.0f", runtime.seconds());
        telemetry.update();
        sleep(2000);
      }
  }
}
