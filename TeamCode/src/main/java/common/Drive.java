package common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
*
*/

public class Drive extends SampleMecanumDrive {

    public Drive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void forward (double distance) {

        List<Double> before = getWheelPositions();
        double beforeHeading = super.getRawExternalHeading();

        Trajectory trajectory = super.trajectoryBuilder(new Pose2d())
                .forward(distance)
                .build();
        super.followTrajectory(trajectory);

        List<Double> after = getWheelPositions();

        Logger.message("distance %6.2f  heading %6.2f  leftFront %6.2f  leftRear %6.2f  rightRear %6.2f  rightFront %6.2f",
                distance,
                super.getRawExternalHeading() - beforeHeading,
                after.get(0) - before.get(0),
                after.get(1) - before.get(1),
                after.get(2) - before.get(2),
                after.get(3) - before.get(3));
    }

    public void back (double distance) {

        List<Double> before = getWheelPositions();
        double beforeHeading = super.getRawExternalHeading();

        Trajectory trajectory = super.trajectoryBuilder(new Pose2d())
                .back(distance)
                .build();
        super.followTrajectory(trajectory);

        List<Double> after = getWheelPositions();

        Logger.message("distance %6.2f  heading %6.2f  leftFront %6.2f  leftRear %6.2f  rightRear %6.2f  rightFront %6.2f",
                distance,
                super.getRawExternalHeading() - beforeHeading,
                after.get(0) - before.get(0),
                after.get(1) - before.get(1),
                after.get(2) - before.get(2),
                after.get(3) - before.get(3));
    }

    public void strafeLeft (double distance) {

        List<Double> before = getWheelPositions();
        double beforeHeading = super.getRawExternalHeading();

        Trajectory trajectory = super.trajectoryBuilder(new Pose2d())
                .strafeLeft(distance)
                .build();
        super.followTrajectory(trajectory);

        List<Double> after = getWheelPositions();

        Logger.message("distance %6.2f  heading %6.2f  leftFront %6.2f  leftRear %6.2f  rightRear %6.2f  rightFront %6.2f",
                distance,
                super.getRawExternalHeading() - beforeHeading,
                after.get(0) - before.get(0),
                after.get(1) - before.get(1),
                after.get(2) - before.get(2),
                after.get(3) - before.get(3));
    }

    public void strafeRight (double distance) {

        List<Double> before = getWheelPositions();
        double beforeHeading = super.getRawExternalHeading();

        Trajectory trajectory = super.trajectoryBuilder(new Pose2d())
                .strafeRight(distance)
                .build();
        super.followTrajectory(trajectory);

        List<Double> after = getWheelPositions();

        Logger.message("distance %6.2f  heading %6.2f  leftFront %6.2f  leftRear %6.2f  rightRear %6.2f  rightFront %6.2f",
                distance,
                super.getRawExternalHeading() - beforeHeading,
                after.get(0) - before.get(0),
                after.get(1) - before.get(1),
                after.get(2) - before.get(2),
                after.get(3) - before.get(3));
    }

    public void turn(double angle) {

        double heading = super.getRawExternalHeading();
        super.turn(angle);
        Logger.message("angle %6.2f  heading %6.2f",
                angle,
                super.getRawExternalHeading() - heading);
    }

}
