
package com.example.meepmeeptesting;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.noahbres.meepmeep.MeepMeep;
        import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
        import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1200);

        RoadRunnerBotEntity redRightRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, -62.5, Math.toRadians(90)))
                        .forward(12)
                        .strafeRight(8.5)
                        .forward(13)
                        .waitSeconds(1)
                        .back(8)
                        .turn(Math.toRadians(-90))
                        .forward(15)
                        .strafeLeft(1)
                        .waitSeconds(5)
                        .build()
                );

        RoadRunnerBotEntity blueRightRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 62.5, Math.toRadians(-90)))
                                .forward(12)
                                .strafeRight(8.5)
                                .forward(13)
                                .waitSeconds(1)
                                .forward(24)
                                .turn(Math.toRadians(90))
                                .forward(77)
                                .strafeLeft(24)
                                .waitSeconds(5)
                                .build()
                );



        RoadRunnerBotEntity redRightLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, -62.5, Math.toRadians(90)))
                                .forward(24)
                                .turn(Math.toRadians(90))
                                .forward(6)
                                .waitSeconds(1)
                                .back(10)
                                .turn(Math.toRadians(180))
                                .forward(22)
                                .strafeLeft(6)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity blueRightLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 62.5, Math.toRadians(-90)))
                                .forward(24)
                                .turn(Math.toRadians(90))
                                .forward(6)
                                .waitSeconds(1)
                                .back(10)
                                .turn(Math.toRadians(-90))
                                .forward(25)
                                .turn(Math.toRadians(90))
                                .forward(77)
                                .strafeLeft(24)
                                .waitSeconds(5)
                                .build()
                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redRightLeft)
                .addEntity(blueRightLeft)
                .start();
    }
}