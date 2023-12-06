package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRightAutonomousTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 60, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-37, 30), Math.toRadians(0))
                                .waitSeconds(1)
                                // spit out pixel here
                                .lineTo(new Vector2d(-34,18))
                                .splineToConstantHeading(new Vector2d(-20, 5), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(24, 15), Math.toRadians(0))
                                .strafeTo(new Vector2d(50,36))
                                .waitSeconds(1)
                                //place pixel on backboard
                                .lineTo(new Vector2d(50, 20))
                                .splineToConstantHeading(new Vector2d(60, 10), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
