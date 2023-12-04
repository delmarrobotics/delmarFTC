
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeft {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        double xStart = -35.25;
        double x2 = 11.75;
        double yStart = -29;
        double y2 = -11.75;

        RoadRunnerBotEntity centerElement = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16,16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(xStart, -62.5, Math.toRadians(90)))
                                .lineTo(new Vector2d(xStart, yStart))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(xStart, y2))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d( x2, y2))
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(x2, -35.25))
                                .turn(Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(centerElement)
                .start();
    }
}