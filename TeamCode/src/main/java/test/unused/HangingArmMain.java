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
@SuppressWarnings("unused")
public class HangingArmMain extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  HangingArm hangingArm = null;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    hangingArm = new HangingArm(this);

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
