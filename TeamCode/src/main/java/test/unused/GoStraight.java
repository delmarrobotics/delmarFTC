package test.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import common.Config;
import common.Logger;
import common.Robot;

/*
 * Test code for the hanging arm
 */
@Disabled
@TeleOp(name="Go Straight", group="Test")
@SuppressWarnings("unused")
public class GoStraight extends LinearOpMode {


    private class Correction {
      double leftFront;
      double rightFront;
      double leftRear;
      double rightRear;

      public Correction(){
        leftFront = 1;
        rightFront = 1;
        leftRear = 1;
        rightRear = 1;
      }

      public Correction(double leftFront, double rightFront, double leftRear, double rightRear) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
      }
    }
  Correction[] corrections;

  /*
  private static final double TICKS_PER_REV = 2000;
  private static final double WHEEL_RADIUS = 0.944882;               // inches 48mm diameter
  static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_RADIUS * 2 * Math.PI);
   */

  static final double COUNTS_PER_MOTOR_REV = 28;              // HD Hex Motor Encoder
  static final double DRIVE_GEAR_REDUCTION = 20;              // Gearing
  static final double WHEEL_DIAMETER_INCHES = (96 / 25.4);     // 96 mm while converted to inches
  static final double COUNTS_PER_INCH =
          (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


  private static final double DECELERATION_COUNT = 4 * COUNTS_PER_INCH;
  private static final double MAX_SPEED = 0.70;
  private static final double MIN_SPEED = 0.10;


  private final ElapsedTime runtime = new ElapsedTime();

  private Robot robot = null;
  List<DcMotorEx> motors;

  public DcMotorEx leftFrontDrive = null;   //  Used to control the left front drive wheel
  public DcMotorEx rightFrontDrive = null;  //  Used to control the right front drive wheel
  public DcMotorEx leftBackDrive = null;    //  Used to control the left back drive wheel
  public DcMotorEx rightBackDrive = null;   //  Used to control the right back drive wheel


  private Encoder leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder;


  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot = new Robot(this);
    robot.init();


    leftFrontDrive = hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
    rightFrontDrive = hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
    leftBackDrive = hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
    rightBackDrive = hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

    leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
    rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
    leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
    rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

    leftFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));   // left rear
    rightFrontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront")); // rightEncoder
    leftRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));     // frontEncoder
    rightRearEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));     // frontEncoder

    leftRearEncoder.setDirection(Encoder.Direction.REVERSE);
    rightRearEncoder.setDirection(Encoder.Direction.REVERSE);

    motors = Arrays.asList(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

    corrections = new Correction[100];

    waitForStart();

    double power = 0.3;

    calculateCorrection(60, power);
    //dumpCorrections();

    //resetEncoders();
    //moveRobotCorrected(60, power, co);
    //displayResults power);


    Correction correction = new Correction(1.0, 0.924, 1.035, 1.01);
    resetEncoders();
    moveRobotCorrected(60, power, correction);
    displayResults (0.2);

/*    moveRobotCorrected(60, power, corrections[20]);
    displayResults (0.2);

*/

    /*
    for (double power = 0.2; power <= 0.2; power += 0.1) {

      resetEncoders();
      moveRobotScaled(60, power, null);

      displayResults (power);
      getCorrection(power);
      if  (! opModeIsActive()) break;
    }
*/


    /*

    for (double power = 0.2; power <= 0.2; power += 0.1) {

      resetEncoders();
      int index = (int)Math.round(power * 100);
      moveRobotScaled(60, power, corrections[index]);

      displayResults (power);
      if  (! opModeIsActive()) break;
    }
*/
  }


  private void displayResults(double power) {

    double leftFrontPos =  (double)leftFrontEncoder.getCurrentPosition();
    double rightFrontPos = (double)rightFrontEncoder.getCurrentPosition();
    double leftRearPos =   (double)leftRearEncoder.getCurrentPosition();
    double rightRearPos =  (double)rightRearEncoder.getCurrentPosition();

    Logger.message("power %5.2f", power);
    Logger.message("left front  %6.0f    %5.2f    %6.3f", leftFrontPos,  (leftFrontPos / COUNTS_PER_INCH),  ((leftFrontPos  / leftFrontPos) - 1) * 100);
    Logger.message("right front %6.0f    %5.2f    %6.3f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), ((rightFrontPos / leftFrontPos) - 1) * 100);
    Logger.message("left rear   %6.0f    %5.2f    %6.3f", leftRearPos,   (leftRearPos / COUNTS_PER_INCH),   ((leftRearPos   / leftFrontPos) - 1) * 100);
    Logger.message("right rear  %6.0f    %5.2f    %6.3f", rightRearPos,  (rightRearPos / COUNTS_PER_INCH),  ((rightRearPos  / leftFrontPos) - 1) * 100);
    Logger.message("\n");
  }

  private void writeCorrection () {
    try {
      File file = new File("filename.txt");
      if (file.exists())
        System.out.println("File already exists." + file.getName());
      else if (file.createNewFile()) {
        System.out.println("File created: " + file.getName());
      }

      FileOutputStream fos = new FileOutputStream(file);
      String content = "This is an example binary content.";
      String str = String.valueOf(corrections[20].leftFront);
      fos.write(str.getBytes());
      fos.close();

    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  private void getCorrection(double power) {

    int index = (int)Math.round(power * 100);
    corrections[index] = new Correction();

    int leftFrontPos = leftFrontEncoder.getCurrentPosition();
    int rightFrontPos = rightFrontEncoder.getCurrentPosition();
    int leftRearPos = leftRearEncoder.getCurrentPosition();
    int rightRearPos = rightRearEncoder.getCurrentPosition();

    corrections[index].leftFront = (double)leftFrontPos / leftFrontPos;
    corrections[index].rightFront = (double)rightFrontPos / leftFrontPos;
    corrections[index].leftRear = (double)leftRearPos / leftFrontPos;
    corrections[index].rightRear = (double)rightRearPos / leftFrontPos;
  }

  private void dumpCorrections() {

    for (int i = 0; i < corrections.length; i++) {
      if (corrections[i] != null) {
        Logger.message("%d %6.3f  %6.3f  %6.3f  %6.3f", i,
                corrections[i].leftFront,
                corrections[i].rightFront,
                corrections[i].leftRear,
                corrections[i].rightRear);
      }
    }
    Logger.message("\n");
  }

  private void resetEncoders () {
    for (DcMotor motor : motors) {
      DcMotor.RunMode mode = motor.getMode();
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor.setMode(mode);
    }

  }

  private void moveRobotCorrected (double inches, double power, Correction correct) {

    double current = 0;

    Encoder encoder = leftFrontEncoder;
    int start = encoder.getCurrentPosition();
    double distance = Math.abs(COUNTS_PER_INCH * inches);

    if (correct == null) {
      leftFrontDrive.setPower(power);
      rightFrontDrive.setPower(power);
      leftBackDrive.setPower(power);
      rightBackDrive.setPower(power);
    } else {
      leftFrontDrive.setPower(power * correct.leftFront);
      rightFrontDrive.setPower(power * correct.rightFront);
      leftBackDrive.setPower(power * correct.leftRear);
      rightBackDrive.setPower(power * correct. rightRear);
    }

    while (opModeIsActive()) {
      current = encoder.getCurrentPosition();
      if (Math.abs(current - start) >= distance) {
        robot.stopRobot();
        break;
      }
    }
    robot.stopRobot();
    //Logger.message("moveRobotScaled: target %f  traveled %f", inches, current / COUNTS_PER_INCH);
  }

  private void calculateCorrection (double inches, double power) {

    double current = 0;
    ElapsedTime elapsedTime = new ElapsedTime();

    Encoder encoder = leftFrontEncoder;
    int start = encoder.getCurrentPosition();
    double distance = Math.abs(COUNTS_PER_INCH * inches);
    Correction correct = new Correction();

    resetEncoders();
    double leftFrontLast = 0;
    double rightFrontLast = 0;
    double leftRearLast = 0;
    double rightRearLast = 0;

    int count = 0;

    elapsedTime.reset();

    while (opModeIsActive()) {

      if (count > 100) break;;
      count++;

      double leftFrontPos =  Math.abs(leftFrontDrive.getCurrentPosition());
      double rightFrontPos = Math.abs(rightFrontDrive.getCurrentPosition());
      double leftRearPos =   Math.abs(leftBackDrive.getCurrentPosition());
      double rightRearPos =  Math.abs(rightBackDrive.getCurrentPosition());

      double leftFrontDelta = (leftFrontPos - leftFrontLast);
      double rightFrontDelta = (rightFrontPos - rightFrontLast);
      double leftRearDelta = (leftRearPos -leftRearLast);
      double rightRearDelta = (rightRearPos - rightRearLast);

      leftFrontLast = leftFrontPos;
      rightFrontLast = rightFrontPos;
      leftRearLast = leftRearPos;
      rightRearLast = rightRearPos;



      //double maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftRearPos), rightRearPos);

      //double avgDelta = (leftFrontDelta + rightFrontDelta + leftRearDelta + rightRearDelta) / 4;

      double avgDelta = rightFrontDelta;


      double adjust;
      if (count < 20)
        adjust = 0.01;
      else if (count < 40)
        adjust = 0.001;
      else if (count < 60)
        adjust = 0.0005;
      else
        adjust = 0.00005;

      if (leftFrontDelta > avgDelta)
        correct.leftFront -= adjust;
      else if (leftFrontDelta < avgDelta) {
        correct.leftFront += adjust;
      }

      if (rightFrontDelta > avgDelta)
        correct.rightFront -= adjust;
      else if (rightFrontDelta < avgDelta) {
        correct.rightFront += adjust;
      }

      if (leftRearDelta > avgDelta)
        correct.leftRear -= adjust;
      else if (leftRearDelta < avgDelta) {
        correct.leftRear += adjust;
      }

      if (rightRearDelta > avgDelta)
        correct.rightRear -= adjust;
      else if (rightRearDelta < avgDelta) {
        correct.rightRear += adjust;
      }
      double leftFrontPower = power * correct.leftFront;
      double rightFrontPower = power * correct.rightFront;
      double leftRearPower = power * correct.leftRear;
      double rightRearPower = power * correct.rightRear;

      Logger.message("%4d    %6.4f    %6.0f  %6.0f  %6.0f  %6.0f     %8.5f  %8.5f  %8.5f  %8.5f     %8.5f  %8.5f  %8.5f  %8.5f",
              count,
              adjust,
              leftFrontDelta,
              rightFrontDelta,
              leftRearDelta,
              rightRearDelta,
              correct.leftFront,
              correct.rightFront,
              correct.leftRear,
              correct.rightRear,
              leftFrontPower,
              rightFrontPower,
              leftRearPower,
              rightRearPower);

      leftFrontDrive.setPower(leftFrontPower);
      rightFrontDrive.setPower(rightFrontPower);
      leftBackDrive.setPower(leftRearPower);
      rightBackDrive.setPower(rightRearPower);

      sleep(2000);

    }
    robot.stopRobot();
    //Logger.message("moveRobotScaled: target %f  traveled %f", inches, current / COUNTS_PER_INCH);

    int index = (int)Math.round(power * 100);
    corrections[index] = new Correction();
    corrections[index] = correct;
  }
}
