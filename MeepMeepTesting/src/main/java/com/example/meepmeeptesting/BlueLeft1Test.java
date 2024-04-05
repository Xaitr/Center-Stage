package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft1Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(14, 61, Math.toRadians(180)))
                                        .strafeTo(new Vector2d(52,42))
                                        //place pixel on backboard
                                        .lineTo(new Vector2d(33, 32))
                                        //place pixel on line
                                        .lineTo(new Vector2d(33, 50))
                                        .splineToConstantHeading(new Vector2d (10,57), Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d (-30,57), Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d (-35,57), Math.toRadians(180))
                                        .splineTo(new Vector2d(-60, 44), Math.toRadians(225))
                                        // pick up white pixels off stack
                                        .splineToConstantHeading(new Vector2d(-50,57), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(-35,57, Math.toRadians(180)),  Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(42,57), Math.toRadians(180))
                                        //place two white pixels
                                        .splineToConstantHeading(new Vector2d (10,57), Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d (-30,57), Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d (-35,57), Math.toRadians(180))
                                        .splineTo(new Vector2d(-60, 44), Math.toRadians(225))
                                        // pick up white pixels off stack
                                        .splineToConstantHeading(new Vector2d(-50,57), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(-35,57, Math.toRadians(180)),  Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(42,57), Math.toRadians(0))
                                        //park and place white pixels
                                        .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}