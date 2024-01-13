/*
 * The class contain support the the autonomous phase of the Center Stage competition
 */
package common;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Auto {
    public enum POSITION { left, center, right }
    public enum COLOR { RED, BLUE }

    boolean PARK_ENABLED = true;
    boolean DROP_PIXEL = true;

    public POSITION objectPosition = POSITION.left;  //ToDo remove, for testing
    COLOR color;

    LinearOpMode opMode;
    Robot robot;

    public Auto(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    public void setColor(COLOR color ) {
        this.color = color;
    }

    public POSITION findTeamElement() {

        String label = null;
        if (color == COLOR.RED)
            label = "red";
        else if (color == COLOR.BLUE)
            label = "blue";

        if (robot.vision.findTeamElement(label,2000)) {
            // Found the team element
            double angle = robot.vision.findTeamElementAngle();
            if (angle < 0) {
                // Team element is on the left spike mark
                objectPosition = POSITION.left;
                Logger.message("Team element %s at left position, angle %f", robot.vision.getElementLabel(), angle);
            } else {
                // Team element is on the center spike mark
                objectPosition = POSITION.center;
                Logger.message("Team element %s at center position, angle %f", robot.vision.getElementLabel(), angle);
            }
        } else {
            // Team element not found, assume the team element is on the right spike mark
            objectPosition = POSITION.right;
            Logger.message("Team element %s at right position", robot.vision.getElementLabel());
        }
        return objectPosition;
    }

    public void dropYellowPixel() {

        if (! DROP_PIXEL) return;

        robot.pixelArm.positionArm(PixelArm.ARM_POSITION.YELLOW);

        robot.drive.moveToObject(1.5,0.25, 10000);

        /*
        boolean found = false;
        if (color == COLOR.BLUE)
            found = robot.drive.moveToColor(Drive.COLOR.BLUE, 1, 0, 0.2, 2000);
        else if (color == COLOR.RED)
            found = robot.drive.moveToColor(Drive.COLOR.RED, 1, 0, 0.2, 2000);
        //adjustYaw();
        if (found)
            robot.forward(11);
         */

        robot.dropYellowPixel();
        robot.back(3);
        robot.pixelArm.pixelWristMove(PixelArm.PIXEL_WRIST_HOME);
        robot.pixelArm.pixelArmMove(PixelArm.PIXEL_ARM_IN);
        robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_DOWN);
    }

    public void parkCorner() {

        if ( ! PARK_ENABLED) return;

        if (color == COLOR.BLUE) {
            if (objectPosition == POSITION.left)
                robot.strafeLeft(18);
            else if (objectPosition == POSITION.center)
                robot.strafeLeft(24);
            else if (objectPosition == POSITION.right)
                robot.strafeLeft(30);
        } else if (color == COLOR.RED) {
            if (objectPosition == POSITION.left)
                robot.strafeRight(30);
            else if (objectPosition == POSITION.center)
                robot.strafeRight(24);
            else if (objectPosition == POSITION.right)
                robot.strafeRight(18 );
        }
        robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_DOWN);
        robot.forward(12);
    }

    public void parkCenter () {
        if (color == COLOR.BLUE) {
            if (objectPosition == POSITION.left)
                robot.strafeRight(30);
            else if (objectPosition == POSITION.center)
                robot.strafeRight(24);
            else if (objectPosition == POSITION.right)
                robot.strafeRight(18 );
        } else if (color == COLOR.RED) {
            if (objectPosition == POSITION.left)
                robot.strafeLeft(18);
            else if (objectPosition == POSITION.center)
                robot.strafeLeft(24);
            else if (objectPosition == POSITION.right)
                robot.strafeLeft(30);
        }
        robot.pixelArm.pixelElbowMove(PixelArm.PIXEL_ELBOW_DOWN);
        robot.forward(12);
    }

    public void adjustYaw() {
        if (robot.vision.findAprilTag(-1)) {
            double x = robot.vision.aprilTagX();
            double range = robot.vision.aprilTagY();
            double yaw = robot.vision.aprilTagYaw();
            Logger.message("aprilTag: x %f  range %f  yaw %f  orientation %3.1f", x, range, yaw, robot.drive.getOrientation());
            robot.turn(yaw);
            Logger.message("robot orientation %3.1f", robot.drive.getOrientation());
        } else {
             Logger.message("no aprilTag found");
        }
    }

    public void adjustYawWithIMU () {
        double angle = 0;
        double yaw = robot.drive.getOrientation();
        if (color == COLOR.BLUE && yaw < 0) {
            angle = yaw + 90;
        } else if (color == COLOR.RED && yaw > 0)
            angle = yaw - 90;
        robot.drive.turn(angle);
    }

    public void strafeToDropPosition () {

        double strafe = 0;

        if (robot.vision.findAprilTag(-1)) {

            //double yaw = robot.vision.aprilTagYaw();
            //robot.drive.turn(yaw);

            double x = robot.vision.aprilTagX();
            int id = robot.vision.aprilTagID();
            if (id == Vision.BLUE_LEFT_TAG || id == Vision.RED_LEFT_TAG) {
                if (objectPosition == POSITION.left) {
                    strafe = x - 6;
                    Logger.message("left tag, left position, strafe %f", strafe);
                } else if (objectPosition == POSITION.center) {
                    strafe = x;
                    Logger.message("left tag, center position, strafe %f", strafe);
                } else {
                    strafe = 6 + x;
                    Logger.message("left tag, right position, strafe %f", strafe);
                }
            } else if (id == Vision.BLUE_CENTER_TAG || id == Vision.RED_CENTER_TAG) {
                if (objectPosition == POSITION.left) {
                    strafe = x - 12;
                    Logger.message("center tag, left position, strafe %f", strafe);
                } else if (objectPosition == POSITION.center) {
                    strafe = x - 6;
                    Logger.message("center tag, center position, strafe %f", strafe);
                } else {
                    strafe = x;
                    Logger.message("center tag, right position, strafe %f", strafe);
                }

            }  else if (id == Vision.BLUE_RIGHT_TAG || id == Vision.RED_RIGHT_TAG) {
                if (objectPosition == POSITION.left) {
                    strafe = x - 18;
                    Logger.message("right tag, left position, strafe %f", strafe);
                } else if (objectPosition == POSITION.center) {
                    strafe = x - 12;
                    Logger.message("right tag, center position, strafe %f", strafe);
                } else {
                    strafe = x - 6;
                    Logger.message("right tag, right position, strafe %f", strafe);
                }
            }

            if (strafe > 0) {
                robot.strafeRight(strafe);
                robot.forward(10);
            }
            else if (strafe < 0 ) {
                robot.strafeLeft(-strafe);
                robot.forward(10);
            }
        }
    }
}

