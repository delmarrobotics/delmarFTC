
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(1200);

        RoadRunnerBotEntity LeftElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, 62.5, Math.toRadians(-90)))
                                .forward(31.25)
                                .turn(Math.toRadians(90))
                                .forward(10)
                                .waitSeconds(1)
                                .forward(26.5)
                                .strafeLeft(10)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity RightElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, 62.5, Math.toRadians(-90)))
                                .forward(27.25)
                                .turn(Math.toRadians(-90))
                                .forward(8)
                                .waitSeconds(1)
                                .back(8)
                                .turn(Math.toRadians(-180))
                                .forward(36.5)
                                .strafeRight(6)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity CenterElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, 62.5, Math.toRadians(-90)))
                                .forward(33.5)
                                .waitSeconds(1)
                                .back(6)
                                .turn(Math.toRadians(90))
                                .forward(36.5)
                                .waitSeconds(5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                //.setBackgroundAlpha(0.95f)
                //.addEntity(LeftElement)
                .addEntity(CenterElement)
                //.addEntity(RightElement)
                .start();
    }
}