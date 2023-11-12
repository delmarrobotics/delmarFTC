package test.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.HangingArm;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Hanging Arm Test", group="Test")
@Disabled
@SuppressWarnings("test/unused")
public class HangingArmMain extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  HangingArm hangingArm = new HangingArm(this);

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      hangingArm.control();

      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.update();
      }
  }
}
