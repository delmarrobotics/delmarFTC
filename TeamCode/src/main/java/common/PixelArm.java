package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
*
*/
public class PixelArm {

    static final double PIXEL_ELBOW_SPEED = .2;
    static final double PIXEL_ARM_SPEED = .2;
    static final int    PIXEL_ELBOW_DOWN = 0;
    static final int    PIXEL_ELBOW_UP = -2974;
    static final int    PIXEL_ARM_IN = 0;
    static final int    PIXEL_ARM_OUT = 2982;
    static final double PIXEL_WRIST_HOME = 0.44;
    static final double PIXEL_WRIST_TARGET = 0.50;
    static final double HAND_UPPER_CLOSED = 0.66 ;
    static final double HAND_UPPER_OPENED = 0.63;
    static final double HAND_LOWER_CLOSED = 0.445;
    static final double HAND_LOWER_OPENED = 0.41;


    private DcMotor pixelElbow = null;
    private DcMotor pixelArm = null;
    private Servo pixelWrist = null;
    private Servo handUpper = null;
    private Servo handLower = null;


    private final ElapsedTime runtime = new ElapsedTime();
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

        runtime.reset();
        while (pixelElbow.isBusy()) {
            if (!opMode.opModeIsActive()) {
                break;
            }
        }
        pixelElbow.setPower(0);
        int to = pixelElbow.getCurrentPosition();
        Logger.message("move elbow from %d to %d, new position %d", from, to, newPosition);
    }

    /**
     * Extend or retract the pixel arm to the specified position. The home position is zero.
     *
     * @param newPosition
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

    /**
     * Manually control the pixel arm
     */
    public boolean control() {

        Gamepad gamepad = opMode.gamepad1;
        boolean handled = true;

        // Raise the hanging arm from its stored position
        if (gamepad.dpad_right) {
            Logger.message("Pixel Elbow Up");
            pixelElbowMove(PIXEL_ELBOW_UP);
        }
        // Lower the hanging arm to its stored position
        else if (gamepad.dpad_left) {
            Logger.message("Pixel Elbow Down");
            pixelElbowMove(PIXEL_ELBOW_DOWN);
        }
        // Extend the telescoping part the the arm
        else if (gamepad.dpad_up) {
            Logger.message("Pixel Arm Out");
            pixelArmMove(PIXEL_ARM_OUT);
        }
        // Retract the telescoping part the the arm
        else if (gamepad.dpad_down) {
            Logger.message("Pixel Arm In");
            pixelArmMove(PIXEL_ARM_IN);
        }
        // Open the upper hand
        else if (gamepad.a) {
            Logger.message("Upper hand open");
            openUpperHand();
        }
        // Close the upper hand
        else if (gamepad.b) {
            Logger.message("Upper hand closed");
            closeUpperHand();
        }
        // Open the lower hand
        else if (gamepad.x) {
            openLowerHand();
        }
        // Close the lower hand
        else if (gamepad.y) {
            closeLowerHand();
        }
        else if (gamepad.left_bumper) {
            pixelWristMove(PIXEL_WRIST_HOME);
        }
        else if (gamepad.right_bumper) {
            pixelWristMove(PIXEL_WRIST_TARGET);
        } else {
            handled = false;
        }
        return handled;
    }
}







