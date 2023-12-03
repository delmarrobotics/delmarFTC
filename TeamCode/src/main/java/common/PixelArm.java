package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
* This file contains the code for the pixel arm.
*/
public class PixelArm {

    public enum DROP_POSITION { LOW, MID, HIGH }

    static final double PIXEL_ELBOW_SPEED = .3;
    static final double PIXEL_ARM_SPEED = .3;

    // Position for all the pixel arm servos and motor encoders
    static final int    PIXEL_ELBOW_DOWN    = 0;
    static final int    PIXEL_ELBOW_COLLECT = -500;
    static final int    PIXEL_ELBOW_UP_LOW  = -2000;
    static final int    PIXEL_ELBOW_UP_MID  = -2000;
    static final int    PIXEL_ELBOW_UP_HIGH = -2000;

    static final int    PIXEL_ARM_IN        = 0;
    static final int    PIXEL_ARM_OUT_LOW   = 1200;
    static final int    PIXEL_ARM_OUT_MID   = 2100;
    static final int    PIXEL_ARM_OUT_HIGH  = 2982;

    static final double PIXEL_WRIST_HOME        = 0.45;
    static final double PIXEL_WRIST_DROP_1_LOW  = 0.34;
    static final double PIXEL_WRIST_DROP_2_LOW  = 0.39;
    static final double PIXEL_WRIST_DROP_1_MID  = 0.34;
    static final double PIXEL_WRIST_DROP_2_MID  = 0.39;
    static final double PIXEL_WRIST_DROP_1_HIGH = 0.34;
    static final double PIXEL_WRIST_DROP_2_HIGH = 0.39;

    static final double HAND_UPPER_CLOSED = 0.66;
    static final double HAND_UPPER_OPENED = 0.635;    // 63
    static final double HAND_LOWER_CLOSED = 0.445;
    static final double HAND_LOWER_OPENED = 0.41;

    private DcMotor pixelElbow = null;
    private DcMotor pixelArm   = null;
    private Servo   pixelWrist = null;
    private Servo   handUpper  = null;
    private Servo   handLower  = null;

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
        Logger.message("Upper hand open");
    }

    public void closeUpperHand() {
        handUpper.setPosition(HAND_UPPER_CLOSED);
        Logger.message("Upper hand close");
    }

    public void toggleUpperHand() {
        if (handUpper.getPosition() == HAND_UPPER_OPENED)
            closeUpperHand();
        else
            openUpperHand();
    }

    public void openLowerHand() {
        handLower.setPosition(HAND_LOWER_OPENED);
        Logger.message("Lower hand open");
    }

    public void closeLowerHand() {
        handLower.setPosition(HAND_LOWER_CLOSED);
        Logger.message("Lower hand close");
    }

    public void toggleLowerHand() {
        if (handLower.getPosition() == HAND_LOWER_OPENED)
            closeUpperHand();
        else
            openUpperHand();
    }

    public void autoDrop(DROP_POSITION position) {
        closeUpperHand();
        closeLowerHand();
        pixelElbowMove(PIXEL_ELBOW_DOWN);
        openUpperHand();
        openLowerHand();
        opMode.sleep(100);

        if (position == DROP_POSITION.LOW) {
            pixelElbowMove(PIXEL_ELBOW_UP_LOW);
            pixelArmMove((PIXEL_ARM_OUT_LOW));
            pixelWristMove(PIXEL_WRIST_DROP_1_LOW);
        } else if (position == DROP_POSITION.MID) {
            pixelElbowMove(PIXEL_ELBOW_UP_MID);
            pixelArmMove((PIXEL_ARM_OUT_MID));
            pixelWristMove(PIXEL_WRIST_DROP_1_MID);
        } else {
            pixelElbowMove(PIXEL_ELBOW_UP_HIGH);
            pixelArmMove((PIXEL_ARM_OUT_HIGH));
            pixelWristMove(PIXEL_WRIST_DROP_1_HIGH);
        }

        opMode.sleep(500);
        closeLowerHand();
        opMode.sleep(500);

        if (position == DROP_POSITION.LOW) {
            pixelWristMove(PIXEL_WRIST_DROP_2_LOW);
        } else if (position == DROP_POSITION.MID) {
            pixelWristMove(PIXEL_WRIST_DROP_2_MID);
        } else {
            pixelWristMove(PIXEL_WRIST_DROP_2_HIGH);
        }

        closeUpperHand();
        pixelWristMove(PIXEL_WRIST_HOME);
        pixelArmMove(PIXEL_ARM_IN);
        pixelElbowMove(PIXEL_ELBOW_COLLECT);
    }

    public boolean dropCommand () {

        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.a || gamepad.b || gamepad.x);
    }

    public void displayControls(){
        opMode.telemetry.addLine("Pixel Arm Controls (Gamepad 2)\n" +
                "  a - drop pixels at low position\n" +
                "  b - drop pixels at mid position\n" +
                "  x - drop pixels at high position\n" +
                "  y - arm down\n" +
                "  left stick - move elbow (u/d)  arm (l/r)\n" +
                "  right stick - manual rotate the hands\n" +
                "  triggers - open/close hands upper(r) lower(l)");
    }

    /**
     * Manually control the pixel arm
     */
    public boolean control() {

        Gamepad gamepad = opMode.gamepad2;
        boolean handled = true;

        if (gamepad.a) {
            // Drop both pixels at the lower position
            autoDrop(DROP_POSITION.LOW);
            while (gamepad.a) opMode.sleep(100);

        } else if (gamepad.b) {
            // Drop both pixels at the middle position
            autoDrop(DROP_POSITION.MID);
            while (gamepad.b) opMode.sleep(100);

        } else if (gamepad.x) {
            // Drop both pixel at the higher position
            autoDrop(DROP_POSITION.HIGH);
            while (gamepad.x) opMode.sleep(100);

        } else if (gamepad.right_trigger != 0) {
            // open / close the upper hand
            toggleUpperHand();
            while (gamepad.right_trigger != 0) opMode.sleep(100);

        } else if (gamepad.left_trigger != 0) {
            // Open / close the lower hand
            toggleLowerHand();
            while (gamepad.left_trigger != 0) opMode.sleep(100);

        } else if (gamepad.dpad_down) {
            // Lower the pixel arm to its stored position
            pixelElbowMove(PIXEL_ELBOW_DOWN);
            Logger.message("Pixel Elbow Down");

        } else if (gamepad.left_stick_y != 0) {
            // manually move the pixel arm elbow
            pixelElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad.left_stick_y > 0)
                pixelElbow.setPower(PIXEL_ELBOW_SPEED);
            else if (gamepad.left_stick_y < 0)
                pixelElbow.setPower(-PIXEL_ELBOW_SPEED);
            while (true) {
                if (gamepad.left_stick_y == 0)
                    break;
            }
            pixelElbow.setPower(0);
            Logger.message( "elbow position %7d", pixelElbow.getCurrentPosition());

        } else if (gamepad.left_stick_x != 0) {
            // manually extend or retract the pixel arm
            pixelArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad.left_stick_x > 0)
                pixelArm.setPower(PIXEL_ARM_SPEED);
            else if (gamepad.left_stick_x < 0)
                pixelArm.setPower(-PIXEL_ARM_SPEED);
            while (true) {
                if (gamepad.left_stick_x == 0)
                    break;
            }
            pixelArm.setPower(0);
            Logger.message( "elbow position %7d", pixelArm.getCurrentPosition());


        } else if (gamepad.right_stick_y != 0) {
            // manually rotate the hands
            while (gamepad.right_stick_y != 0) {
                double position = pixelWrist.getPosition();
                if (gamepad.right_stick_y > 0)
                    position += 0.005;
                else if (gamepad.right_stick_y < 0)
                    position -= 0.005;
                pixelWrist.setPosition(position);
                opMode.sleep(100);
            }
            Logger.message("wrist position %f", pixelWrist.getPosition());

        } else {
            handled = false;
        }

        return handled;
    }
}







