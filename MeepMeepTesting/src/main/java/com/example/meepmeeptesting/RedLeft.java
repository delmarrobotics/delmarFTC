
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeft {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(750);

        double xStart = -35.25;
        double x2 = 11.75;
        double yStart = -29;
        double y2 = -11.75;

        RoadRunnerBotEntity LeftElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, -62.5, Math.toRadians(90)))
                                .strafeLeft(13)
                                .lineTo(new Vector2d(-48.25, -31))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-48.25, -11.75))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d( 11.75, -11.75))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(11.75, -29.25))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(47.75,-29.25))
                                .build()
                );
        RoadRunnerBotEntity RightElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, -62.5, Math.toRadians(90)))
                                .lineTo(new Vector2d(-35.25, -33))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(-30,-33))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(47.25, -41.25))
                                .build()
                );
        RoadRunnerBotEntity CenterElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, -62.5, Math.toRadians(90)))
                                .lineTo(new Vector2d(-35.25, -29))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-35.25, -11.75))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d( 11.75, -11.75))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(11.75, -35.25))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(47.75,-35.25))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(RightElement)
                .start();
    }
}