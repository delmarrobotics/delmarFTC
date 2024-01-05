/*
 * Test code for threads
 */
package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import test.unused.ThreadChild;

@TeleOp(name="Thread Test", group="Test")
@SuppressWarnings("unused")
public class DriveTest extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();
  //Logger logger = Logger.getLogger("MyLog");
  ThreadChild child  = null;

  @Override
  public void runOpMode() {
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      telemetry.addData("Status", "Press start");
      telemetry.update();

      Drive drive = new Drive(this);
      drive.start();

      waitForStart();

      boolean found;

      runtime.reset();

      while (opModeIsActive()) {

        if (gamepad1.a)
            while (gamepad1.a) sleep(100);

        if (gamepad1.x)
             found = drive.moveToObject(3,0.25, 3000);
      }
  }
}
