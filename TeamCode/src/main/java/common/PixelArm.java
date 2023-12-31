/*
 * This file contains the code for the pixel arm.
 */

package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PixelArm {

    public enum ARM_POSITION { HOME, LOW, MID, HIGH, YELLOW }
    private enum PIXEL_ARM_STATE { NONE, MOVE_HOME, MOVE_YELLOW, MOVE_LOW, MOVE_MID, MOVE_HIGH, DROP_PIXEL, ARM_UP }

    static final double PIXEL_ELBOW_SPEED = .75;
    static final double PIXEL_ARM_SPEED = .5;

    // Position for all the pixel arm servos and motor encoders
    public static final int    PIXEL_ELBOW_DOWN      = 0;
    public static final int    PIXEL_ELBOW_UP_YELLOW = -1750;  //-2000;
    public static final int    PIXEL_ELBOW_UP_LOW    = -1870;  //-2000;
    public static final int    PIXEL_ELBOW_UP_MID    = -2230;
    public static final int    PIXEL_ELBOW_UP_HIGH   = -2700;

    public static final int    PIXEL_ARM_IN          = 0;
    public static final int    PIXEL_ARM_OUT_YELLOW  = 1080;
    public static final int    PIXEL_ARM_OUT_LOW     = 1280;
    public static final int    PIXEL_ARM_OUT_MID     = 2100;
    public static final int    PIXEL_ARM_OUT_HIGH    = 2982;

    public static final double PIXEL_WRIST_HOME         = 0.64;
    public static final double PIXEL_WRIST_DROP_YELLOW  = 0.44;   // 0.48;
    public static final double PIXEL_WRIST_DROP_LOW     = 0.44;   // 0.48;
    public static final double PIXEL_WRIST_DROP_MID     = 0.445;
    public static final double PIXEL_WRIST_DROP_HIGH    = 0.445;

    private DcMotor pixelElbow = null;
    private DcMotor pixelArm   = null;
    private Servo   pixelWrist = null;

    private PIXEL_ARM_STATE state  = PIXEL_ARM_STATE.NONE;
    private final ElapsedTime stateTime = new ElapsedTime();

    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;

    private boolean pixelArmActive = false;

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

            pixelArm.setDirection(DcMotor.Direction.REVERSE);
            pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pixelArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pixelArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pixelElbow.setDirection(DcMotor.Direction.REVERSE);
            pixelElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pixelElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pixelElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            pixelWrist.setPosition(PIXEL_WRIST_HOME);
            pixelArmMove(PIXEL_ARM_IN);

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

        /*
        while (pixelElbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        pixelElbow.setPower(0);
        */

        Logger.message("move elbow from %d to %d", from, newPosition);
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

        /*
        while (pixelArm.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }

        if (newPosition == 0) {
            // only stop the motor when the arm is lowered
            pixelArm.setPower(0);
        }
        */
    }

    public void pixelWristMove(double position) {
        pixelWrist.setPosition(position);
    }

    public void enablePixelArm (boolean enable) {
        pixelArmActive = enable;
    }


    public void run () {

        if (pixelArmActive)
            control();

        if (state == PIXEL_ARM_STATE.NONE)
            return;

        if (state == PIXEL_ARM_STATE.MOVE_YELLOW) {
            if (stateTime.milliseconds() < 1000)
                return;
            pixelArmMove((PIXEL_ARM_OUT_YELLOW));
            pixelWristMove(PIXEL_WRIST_DROP_YELLOW);
            state = PIXEL_ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to yellow position");

        } else if (state == PIXEL_ARM_STATE.MOVE_LOW) {
            if (stateTime.milliseconds() < 500)
                return;
            pixelArmMove((PIXEL_ARM_OUT_LOW));
            pixelWristMove(PIXEL_WRIST_DROP_LOW);
            state = PIXEL_ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to low position");

        } else if (state == PIXEL_ARM_STATE.MOVE_MID) {
            if (stateTime.milliseconds() < 500)
                return;
            pixelArmMove((PIXEL_ARM_OUT_MID));
            pixelWristMove(PIXEL_WRIST_DROP_MID);
            state = PIXEL_ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to middle position");

        } else if (state == PIXEL_ARM_STATE.MOVE_HIGH) {
            if (stateTime.milliseconds() < 500)
                return;
            pixelArmMove((PIXEL_ARM_OUT_HIGH));
            pixelWristMove(PIXEL_WRIST_DROP_HIGH);
            state = PIXEL_ARM_STATE.NONE;
            Logger.message("pixel arm and wrist to high position");

        } else if (state == PIXEL_ARM_STATE.MOVE_HOME) {
            if (stateTime.milliseconds() < 1000)
                return;
            pixelElbowMove(PIXEL_ELBOW_DOWN);
            Logger.message("pixel elbow to home position");

            if (pixelArm.isBusy())
                return;
            pixelArm.setPower(0);
            state = PIXEL_ARM_STATE.NONE;
            Logger.message("pixel arm power off");

        } else if (state == PIXEL_ARM_STATE.ARM_UP) {
            if (stateTime.milliseconds() < 10000)
                return;
            pixelArm.setPower(0);
            state = PIXEL_ARM_STATE.NONE;
        }
    }

    public void dropPixel () {
        state = PIXEL_ARM_STATE.DROP_PIXEL;
        stateTime.reset();
    }

    public void positionArmAsyn(ARM_POSITION position) {

        stateTime.reset();
        if (position == ARM_POSITION.YELLOW) {
            pixelElbowMove(PIXEL_ELBOW_UP_YELLOW);
            state = PIXEL_ARM_STATE.MOVE_YELLOW;
        } else if (position == ARM_POSITION.LOW) {
            pixelElbowMove(PIXEL_ELBOW_UP_LOW);
            Logger.message("pixel elbow to low position");
            state = PIXEL_ARM_STATE.MOVE_LOW;
        } else if (position == ARM_POSITION.MID) {
            pixelElbowMove(PIXEL_ELBOW_UP_MID);
            state = PIXEL_ARM_STATE.MOVE_MID;
            Logger.message("pixel elbow to middle position");
        } else if (position == ARM_POSITION.HIGH) {
            pixelElbowMove(PIXEL_ELBOW_UP_HIGH);
            state = PIXEL_ARM_STATE.MOVE_HIGH;
            Logger.message("pixel elbow to high position");
        } else if (position == ARM_POSITION.HOME) {
            pixelWristMove(PIXEL_WRIST_HOME);
            pixelArmMove(PIXEL_ARM_IN);
            state = PIXEL_ARM_STATE.MOVE_HOME;
            Logger.message("pixel wrist and arm to home position");
        }
    }

    public void positionArm(ARM_POSITION position) {

        if (position == ARM_POSITION.YELLOW) {
            pixelElbowMove(PIXEL_ELBOW_UP_YELLOW);
            opMode.sleep(500);
            pixelArmMove((PIXEL_ARM_OUT_YELLOW));
            pixelWristMove(PIXEL_WRIST_DROP_YELLOW);
        } else if (position == ARM_POSITION.LOW) {
            pixelElbowMove(PIXEL_ELBOW_UP_LOW);
            opMode.sleep(500);
            pixelArmMove((PIXEL_ARM_OUT_LOW));
            pixelWristMove(PIXEL_WRIST_DROP_LOW);
        } else if (position == ARM_POSITION.MID) {
            pixelElbowMove(PIXEL_ELBOW_UP_MID);
            pixelArmMove((PIXEL_ARM_OUT_MID));
            pixelWristMove(PIXEL_WRIST_DROP_MID);
        } else if (position == ARM_POSITION.HIGH) {
            pixelElbowMove(PIXEL_ELBOW_UP_HIGH);
            pixelArmMove((PIXEL_ARM_OUT_HIGH));
            pixelWristMove(PIXEL_WRIST_DROP_HIGH);
        } else if (position == ARM_POSITION.HOME) {
            pixelWristMove(PIXEL_WRIST_HOME);
            pixelArmMove(PIXEL_ARM_IN);
            pixelElbowMove(PIXEL_ELBOW_DOWN);
            while (pixelArm.isBusy()) {
                if (!opMode.opModeIsActive()) {
                    break;
                }
            }
            pixelArm.setPower(0);
        }
    }

    public boolean positionCommand () {

        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.a || gamepad.b || gamepad.x);
    }

    public boolean dropCommand () {
        Gamepad gamepad = opMode.gamepad2;
        return (gamepad.right_trigger > 0);

    }

    public void displayControls(){
        opMode.telemetry.addLine("Pixel Arm Controls (Gamepad 2)\n" +
                "  a - position arm at low position\n" +
                "  b - position arm at mid position\n" +
                "  x - position arm at high position\n" +
                "  y - position arm at pickup position\n" +
                "  right triggers - drop pixels\n"
                //"  left stick - move elbow (u/d)  arm (l/r)\n" +
                //"  right stick - manual rotate the hands\n"
                );
    }

    /**
     * Manually control the pixel arm
     */
    public boolean control() {

        Gamepad gamepad = opMode.gamepad2;
        boolean handled = true;

        run();   // ToDo remove when subclass from thread

        if (gamepad.a) {
            // Move the arm to the lower drop position
            if (! aPressed) {
                positionArmAsyn(ARM_POSITION.LOW);
                aPressed = true;
            }
        } else {
            aPressed = false;
        }

        if (gamepad.b) {
            // Move the arm to the middle drop position
            if (!bPressed) {
                positionArmAsyn(ARM_POSITION.MID);
                bPressed = true;
            }
        } else {
            bPressed = false;
        }

        if (gamepad.x) {
            // Move the arm to the higher drop position
            if (!xPressed) {
                positionArmAsyn(ARM_POSITION.HIGH);
                xPressed = true;
            }
        } else {
            xPressed = false;
        }

        if (gamepad.y) {
            // Move the arm to the higher drop position
            if (!yPressed) {
                positionArmAsyn(ARM_POSITION.HOME);
                yPressed = true;
            }
        } else {
            yPressed = false;
        }

        if (gamepad.left_stick_y != 0) {
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
            int position = pixelArm.getCurrentPosition();
            pixelArm.setTargetPosition(position);
            pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Logger.message( "arm position %7d", position);

        } else if (gamepad.right_stick_y != 0) {
            // manually rotate the bucket
            while (gamepad.right_stick_y != 0) {
                double position = pixelWrist.getPosition();
                if (Double.isNaN(position))
                    position = PIXEL_WRIST_HOME;
                else if (gamepad.right_stick_y > 0)
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







