/*
 * Test code for threads
 */
package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import common.PixelArm;
import common.Robot;
import test.unused.ThreadChild;

@TeleOp(name=" Load Pixels", group="Test")
public class LoadPixels extends LinearOpMode {


    @Override
  public void runOpMode() {

      Robot robot = new Robot(this);
      robot.init();

      robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_UP_HIGH);
      robot.dropper.setPosition(Robot.DROPPER_OPEN);

      telemetry.addLine("Load pixel, then press start");
      telemetry.update();

      waitForStart();

      robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_DOWN);

      ElapsedTime runtime = new ElapsedTime();
      runtime.reset();

      while (opModeIsActive()) {
        if (runtime.seconds() > 3)
            break;
      }
  }
}
