package test.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.PixelArm;

/*
 * Test code for the pixel arm
 */
@TeleOp(name="Pixel Arm Test", group="Test")
@Disabled
@SuppressWarnings("unused")
public class PixelArmTest extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  PixelArm pixelArm = null;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    pixelArm = new PixelArm(this);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    pixelArm.displayControls();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      pixelArm.control();

//      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.update();
      }
  }
}
