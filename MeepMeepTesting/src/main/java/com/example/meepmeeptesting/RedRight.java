package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, -61, Math.toRadians(180)))
                                .turn(0)
                               // .lineToConstantHeading(new Vector2d(14, -34))
                               // .splineToConstantHeading(new Vector2d(26, -32), Math.toRadians(0))
                                .strafeTo(new Vector2d(50,-32))
                                //.splineTo(new Vector2d(50, -32), Math.toRadians(0))
                                .turn(Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(50, -34))
                                .splineToConstantHeading(new Vector2d(56, -60), Math.toRadians(0))
                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}