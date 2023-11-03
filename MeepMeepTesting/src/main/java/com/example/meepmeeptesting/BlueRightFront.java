package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRightFront {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 57, Math.toRadians(90)))
                                .turn(0)
                                .lineTo(new Vector2d(-33, 30))
                                .turn(Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(-33, 11))
                                .splineToConstantHeading(new Vector2d(-32, 10), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-16, 10), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(38, 10), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, 34), Math.toRadians(0))
                                .turn(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(58, 14), Math.toRadians(0))



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}