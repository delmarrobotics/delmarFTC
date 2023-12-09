package common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
*
*/
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
     * Drop the purple pixel
     *
     * @param x forward  1, backward -1
     * @param y right 1, left -1
     */
    public void purplePixel (double x, double y) {

        if (color == COLOR.RED)
            robot.moveToColor(Robot.COLOR.RED, x, y, 0.2,3000);
        else
            robot.moveToColor(Robot.COLOR.BLUE, x, y, 0.2,3000);
        robot.moveRobot(x, y, 0, 0.2);
        opMode.sleep(200);
        robot.stopRobot();
        robot.dropPurplePixel();
    }

    public void yellowPixel () {

        int tag;
        if (color == COLOR.BLUE)
            tag = Vision.BLUE_LEFT_TAG;
        else
            tag = Vision.BLUE_LEFT_TAG; //ToDo need color param

        while (opMode.opModeIsActive()) {
            if (robot.vision.findAprilTag(tag)) {
                double x = robot.vision.aprilTagX();
                double range = robot.vision.aprilTagY();
                double yaw = robot.vision.aprilTagYaw();
                Logger.message("x %f  range %f  yaw %f", x, range, yaw);

                TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                        .turn(Math.toRadians(yaw))
                        //.forward(range-2)
                        .build();
                drive.followTrajectorySequence(traj1);

                Logger.message("robot orientation %3.1f", robot.getOrientation());

                //robot.moveToColor(Robot.COLOR.RED, 1, 0, 0.2,3000);
                //robot.moveToColor(Robot.COLOR.BLUE, 1, 0, MIN_SPEED, 4000);

                double strafe = 0;
                if (objectPosition == POSITION.left) {
                    strafe = x - 6;
                    Logger.message("left, strafe %f", strafe);
                } else if (objectPosition == POSITION.center) {
                    strafe = x;
                    Logger.message("center, strafe %f", strafe);
                } else {
                    strafe = 6 + x;
                    Logger.message("right, strafe %f", strafe);
                }

                Trajectory traj2;
                if (strafe > 0) {
                    traj2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeRight(x)
                            .build();
                    drive.followTrajectory(traj2);
                }
                else if (strafe < 0 ) {
                    traj2 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(-strafe)
                            .build();
                    drive.followTrajectory(traj2);
                }
                break;

            } else {
                Logger.message("Tag not found");
            }
        }
    }
}

