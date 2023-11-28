package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
* This file contains the code for the pixel arm.
*/
public class PixelArm {

    static final double PIXEL_ELBOW_SPEED = .3;
    static final double PIXEL_ARM_SPEED = .3;

    // Position for all the pixel arm servos and motor encoders
    static final int    PIXEL_ELBOW_DOWN = 0;
    static final int    PIXEL_ELBOW_COLLECT = -500;
    static final int    PIXEL_ELBOW_UP = -2000;

    static final int    PIXEL_ARM_IN = 0;
    static final int    PIXEL_ARM_OUT = 1200; // ToDo fully extended 2982;

    static final double PIXEL_WRIST_HOME = 0.45;
    static final double PIXEL_WRIST_TARGET = 0.52;
    static final double PixEL_WRIST_DROP_1 = 0.34;
    static final double PixEL_WRIST_DROP_2 = 0.39;

    static final double HAND_UPPER_CLOSED = 0.66 ;
    static final double HAND_UPPER_OPENED = 0.635;    // 63
    static final double HAND_LOWER_CLOSED = 0.445;
    static final double HAND_LOWER_OPENED = 0.41;

    private DcMotor pixelElbow = null;
    private DcMotor pixelArm = null;
    private Servo pixelWrist = null;
    private Servo handUpper = null;
    private Servo handLower = null;

    public LinearOpMode opMode;

    public PixelArm(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void init() {
        try {
            pixelArm = opMode.hardwareMap.get(DcMotor.class, Config.PIXEL_ARM);
            pixelElbow = opMode.hardwareMap.get(DcMotor.class, Config.PIXEL_ELBOW);
            pixelWrist = opMode.hardwareMap.get(Servo.class, Config.PIXEL_WRIST);
            handUpper = opMode.hardwareMap.get(Servo.class, Config.HAND_UPPER);
            handLower = opMode.hardwareMap.get(Servo.class, Config.HAND_LOWER);

            //2982 in extended and 0 at default
            pixelArm.setDirection(DcMotor.Direction.REVERSE);
            pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pixelArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pixelArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pixelElbow.setDirection(DcMotor.Direction.REVERSE);
            pixelElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pixelElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pixelElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pixelWrist.setPosition(PIXEL_WRIST_HOME);
            handUpper.setPosition(HAND_UPPER_CLOSED);
            handLower.setPosition(HAND_LOWER_CLOSED);

        } catch (Exception e) {
            Logger.error(e, "Pixel arm hardware not found");
        }
    }

    /**
     * Move the pixel arm elbow to the specified position
     *
     * @param newPosition position to move to
     */
    public void pixelElbowMove(int newPosition) {

        int from = pixelElbow.getCurrentPosition();
        pixelElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixelElbow.setTargetPosition(newPosition);
        pixelElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelElbow.setPower(Math.abs(PIXEL_ELBOW_SPEED));

        while (pixelElbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        pixelElbow.setPower(0);
        int to = pixelElbow.getCurrentPosition();
        Logger.message("move elbow from %d to %d, new position %d", from, to, newPosition);
    }

    public void pixelElbowCollectPosition () {
        pixelElbowMove(PIXEL_ELBOW_COLLECT);
    }

    /**
     * Extend or retract the pixel arm to the specified position. The home position is zero.
     *
     * @param newPosition position to move to
     */
    public void pixelArmMove(int newPosition) {

        pixelArm.setTargetPosition(newPosition);
        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(Math.abs(PIXEL_ARM_SPEED));

        while (pixelArm.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }

        if (newPosition == 0) {
            // only stop the motor when the arm is lowered
            pixelArm.setPower(0);
        }
    }

    public void pixelWristMove(double position) {
        pixelWrist.setPosition(position);
    }

    public void openUpperHand() {
        handUpper.setPosition(HAND_UPPER_OPENED);
    }

    public void closeUpperHand() {
        handUpper.setPosition(HAND_UPPER_CLOSED);
    }

    public void openLowerHand() {
        handLower.setPosition(HAND_LOWER_OPENED);
    }

    public void closeLowerHand() {
        handLower.setPosition(HAND_LOWER_CLOSED);
    }

    public void displayControls(){
        opMode.telemetry.addData("Pixel Arm Controls (Gamepad 2)", "\n" +
                "  dpad up - arm up\n" +
                "  dpad down - arm down\n" +
                "  dpad left - rotate hands up\n" +
                "  dpad right - rotate hands down\n" +
                "  left bumper - extend arm\n" +
                "  right bumper - retract arm\n" +
                "  left stick - manual move the elbow\n" +
                "  right stick - manual rotate the hands\n" +
                "  a - close lower hand\n" +
                "  b - open lower hand\n" +
                "  x - close upper hand\n" +
                "  y - open upper hand\n" +
                "  left trigger - pixel pickup / drop test" +
                "\n");
    }

    /**
     * Manually control the pixel arm
     */
    public boolean control() {

        Gamepad gamepad = opMode.gamepad2;
        boolean handled = true;

        if (gamepad.dpad_up) {
            // Raise the pixel arm from its stored position
            Logger.message("Pixel Elbow Up");
            pixelElbowMove(PIXEL_ELBOW_UP);

        } else if (gamepad.dpad_down) {
            // Lower the pixel arm to its stored position
            Logger.message("Pixel Elbow Down");
            pixelElbowMove(PIXEL_ELBOW_DOWN);

        } else if (gamepad.dpad_right) {
            // Rotate the hands up
            Logger.message("Rotate hands up");
            pixelWristMove(PIXEL_WRIST_TARGET);

        }  else if (gamepad.dpad_left) {
            // rotate the hands down
            Logger.message("Rotate hands down");
            pixelWristMove(PIXEL_WRIST_HOME);

        } else if (gamepad.left_bumper) {
            // Retract the telescoping part the the arm
            Logger.message("Pixel Arm In");
            pixelArmMove(PIXEL_ARM_IN);

        } else if (gamepad.right_bumper) {
            // Extend the telescoping part the the arm
            Logger.message("Pixel Arm Out");
            pixelArmMove(PIXEL_ARM_OUT);

        } else if (gamepad.y) {
            // Open the upper hand
            Logger.message("Upper hand opened");
            openUpperHand();

        } else if (gamepad.x) {
            // Close the upper hand
            Logger.message("Upper hand closed");
            closeUpperHand();

        } else if (gamepad.b) {
            Logger.message("Lower hand opened");
            // Open the lower hand
            openLowerHand();

        } else if (gamepad.a) {
            // Close the lower hand
            Logger.message("Lower hand closed");
            closeLowerHand();

        } else if (gamepad.left_stick_y > 0) {
            // manually move the pixel arm elbow
            Logger.message( "elbow position %7d", pixelElbow.getCurrentPosition());
            pixelElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pixelElbow.setPower(PIXEL_ELBOW_SPEED);
            while (true) {
                if (gamepad.left_stick_y <= 0)
                    break;
            }
            pixelElbow.setPower(0);
            opMode.sleep(200);

        } else if (gamepad.left_stick_y < 0) {
            // manually move the pixel arm elbow
            Logger.message( "elbow position %7d", pixelElbow.getCurrentPosition());
            pixelElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pixelElbow.setPower(-PIXEL_ELBOW_SPEED);
            while (true) { if (gamepad.left_stick_y >= 0) break; }
            pixelElbow.setPower(0);
            opMode.sleep(200);

        } else if (gamepad.right_stick_y > 0) {
            // manually rotate the hands
            while (gamepad.right_stick_y > 0) {
                double position = pixelWrist.getPosition() + .005;
                pixelWrist.setPosition(position);
                Logger.message("wrist position %f", position);
                opMode.sleep(100);
            }

        } else if (gamepad.right_stick_y < 0) {
            while (gamepad.right_stick_y < 0) {
                double position = pixelWrist.getPosition() - .005;
                pixelWrist.setPosition(position);
                Logger.message("wrist position %f", position);
                opMode.sleep(100);
            }

        } else {
            handled = false;
        }

        if (gamepad.left_trigger != 0 ){
            pixelElbowMove(PIXEL_ELBOW_DOWN);
            openUpperHand();
            openLowerHand();
            opMode.sleep(100);
            pixelElbowMove(PIXEL_ELBOW_UP);
            pixelArmMove((PIXEL_ARM_OUT));
            pixelWristMove(PixEL_WRIST_DROP_1);
            opMode.sleep(500);
            closeLowerHand();
            opMode.sleep(500);
            pixelWristMove(PixEL_WRIST_DROP_2);
            closeUpperHand();
            pixelWristMove(PIXEL_WRIST_HOME);
            pixelArmMove(PIXEL_ARM_IN);
            pixelElbowMove(PIXEL_ELBOW_COLLECT);
        }

        if (gamepad.right_trigger != 0) {
            pixelWristMove(PIXEL_WRIST_HOME);
            pixelArmMove(PIXEL_ARM_IN);
            pixelElbowMove(PIXEL_ELBOW_COLLECT);

        }
        return handled;
    }
}







