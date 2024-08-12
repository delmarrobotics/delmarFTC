
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRight {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(1200);

        RoadRunnerBotEntity LeftElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 62.5, Math.toRadians(-90)))
                                .forward(27.25)
                                .turn(Math.toRadians(-90))
                                .back(15.75)
                                .waitSeconds(1)
                                .forward(15.75)
                                .turn(Math.toRadians(90))
                                .forward(23.5)
                                .turn(Math.toRadians(90))
                                .forward(70.5)
                                .strafeLeft(29.5)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity RightElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 62.5, Math.toRadians(-90)))
                                .forward(27.25)
                                .turn(Math.toRadians(-90))
                                .forward(8)
                                .waitSeconds(1)
                                .back(8)
                                .turn(Math.toRadians(90))
                                .forward(23.5)
                                .turn(Math.toRadians(90))
                                .forward(70.5)
                                .strafeLeft(17.5)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity CenterElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, 62.5, Math.toRadians(-90)))
                                .forward(33.5)
                                .waitSeconds(1)
                                .forward(17.25)
                                .turn(Math.toRadians(90))
                                .forward(70.5)
                                .strafeLeft(23.5)
                                .waitSeconds(5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(LeftElement)
                //.addEntity(RightElement)
                //.addEntity(CenterElement)
                .start();
    }
}