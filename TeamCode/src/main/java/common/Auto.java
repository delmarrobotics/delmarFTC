/*
 * The class contain support the the autonomous phase of the Center Stage competition
 */
package common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Auto {
    public enum POSITION { left, center, right }
    public enum COLOR { RED, BLUE }

    POSITION objectPosition;
    COLOR color;

    LinearOpMode opMode;
    Robot robot;
    SampleMecanumDrive drive;

    public Auto(LinearOpMode opMode, Robot robot, SampleMecanumDrive drive) {
        this.opMode = opMode;
        this.robot = robot;
        this.drive = drive;
    }

    public void setColor(COLOR color ) {
        this.color = color;
    }

    public POSITION findTeamElement() {

        if (!robot.vision.findTeamElement(3000)) {
            // Team element not found
            objectPosition = POSITION.right;
            Logger.message("Team element at right position");
        } else {
            // Team element is on the left spike mark
            double angle = robot.vision.findTeamElementAngle();
            objectPosition = POSITION.left;
            if (angle < 0) {
                Logger.message("Team element at left position, angle %f", angle);
            } else {
                // Team element is on the center spike mark
                Logger.message("Team element at center position, angle %f", angle);
                objectPosition = POSITION.center;
            }
        }
        return objectPosition;
    }

    /**
     * Drop the purple pixel on the spike mark.
     *
     * @param x direction to search, forward  1, backward -1
     * @param y direction to search, right 1, left -1
     */
    public void purplePixel (double x, double y) {

        if (color == COLOR.RED)
            robot.moveToColor(Robot.COLOR.RED, x, y, 0.2,3000);
        else
            robot.moveToColor(Robot.COLOR.BLUE, x, y, 0.2,3000);
        robot.moveDistance(robot.MIN_SPEED, 1.5, 1.5, 2000);
        robot.dropPurplePixel();
    }

    /**
     * Drop the yellow pixel on the backdrop.
     */
    public void yellowPixel () {

        double strafe = 0;
        int retry = 0;

        while (opMode.opModeIsActive()) {
            if (robot.vision.findAprilTag(-1)) {
                double x = robot.vision.aprilTagX();
                double range = robot.vision.aprilTagY();
                double yaw = robot.vision.aprilTagYaw();
                Logger.message("x %f  range %f  yaw %f", x, range, yaw);

                TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                        .turn(Math.toRadians(yaw))
                        .forward(range-2)
                        .build();
                drive.followTrajectorySequence(traj1);
                Logger.message("robot orientation %3.1f", robot.getOrientation());

                if (color == COLOR.BLUE)
                    robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.2,3000);
                else if (color == COLOR.RED)
                    robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.2,3000);

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
                        Logger.message("center tag, left position, strafe %f", strafe);
                    } else if (objectPosition == POSITION.center) {
                        strafe = x - 12;
                        Logger.message("center tag, center position, strafe %f", strafe);
                    } else {
                        strafe = x - 6;
                        Logger.message("center tag, right position, strafe %f", strafe);
                    }
                }

                Trajectory traj2;
                if (strafe > 0) {
                    traj2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(strafe)
                            .build();
                    drive.followTrajectory(traj2);
                }
                else if (strafe < 0 ) {
                    traj2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(-strafe)
                            .build();
                    drive.followTrajectory(traj2);
                }

                // robot.dropYellowPixel();  //ToDo uncomment
                break;

            } else {
                Logger.message("Tag not found");
                if (retry == 0) {
                    robot.moveDistance(robot.MIN_SPEED, -3, -3, 2000);
                    retry += 1;
                }
            }
        }
    }
}

