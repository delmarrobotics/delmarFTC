
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedRight {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(1200);

        RoadRunnerBotEntity CenterElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, -62.5, Math.toRadians(90)))
                                .forward(33.5)
                                .waitSeconds(1)
                                .back(6)
                                .turn(Math.toRadians(-90))
                                .forward(36.5)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity LeftElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, -62.5, Math.toRadians(90)))
                                .forward(27.25)
                                //.lineTo(new Vector2d(11.75, -35.25))
                                .turn(Math.toRadians(-90))
                                .back(15.75)
                                .waitSeconds(1)
                                .forward(52.25)
                                .strafeLeft(6)
                                .waitSeconds(5)
                                .build()
                );

        RoadRunnerBotEntity RightElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, -62.5, Math.toRadians(90)))
                                .strafeRight(13)
                                .forward(31.5)
                                .waitSeconds(1)
                                .back(10)
                                .turn(Math.toRadians(-90))
                                .forward(23.5)
                                .waitSeconds(5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(LeftElement)
                .addEntity(CenterElement)
                .addEntity(RightElement)
                .start();
    }
}