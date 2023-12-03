package common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
*
*/
public class Navigate {

    private SampleMecanumDrive drive = null;
    private Pose2d pose;
    private LinearOpMode opMode;


    //constructor
    public Navigate(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    private void init () {
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        pose = new Pose2d();
    }

    public void setPose(double x, double y, double heading) {
        pose.copy(x, y, heading);
    }

    public void forward(double inches) {

        Trajectory trajectory = drive.trajectoryBuilder(pose)
                .forward(inches)
                .build();
        drive.followTrajectory(trajectory);
        pose = trajectory.end();
    }

    public void back(double inches) {

        Trajectory trajectory = drive.trajectoryBuilder(pose)
                .back(inches)
                .build();
        drive.followTrajectory(trajectory);
        pose = trajectory.end();
    }

    public void strafeLeft(double inches) {

        Trajectory trajectory = drive.trajectoryBuilder(pose)
                .strafeLeft(inches)
                .build();
        drive.followTrajectory(trajectory);
        pose = trajectory.end();
    }

    public void strafeRight(double inches) {

        Trajectory trajectory = drive.trajectoryBuilder(pose)
                .strafeRight(inches)
                .build();
        drive.followTrajectory(trajectory);
        pose = trajectory.end();
    }

}
