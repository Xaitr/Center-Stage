package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeft1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(180)))
                                .lineTo(new Vector2d(-36, -30))
                                .turn(0)
                                //place pixel on line
                                .lineTo(new Vector2d(-34,-18))
                                .splineToConstantHeading(new Vector2d(-20,-8), Math.toRadians(0))
                                .lineTo(new Vector2d(-55,-8))
                                .splineToConstantHeading(new Vector2d(-14, -8), Math.toRadians(0))
                                .lineTo(new Vector2d(30,-8))
                                .splineToConstantHeading(new Vector2d(50,-32), Math.toRadians(0))
                                .turn (0)
                                //place pixel on backboard
                                .splineToConstantHeading(new Vector2d(30,-8), Math.toRadians(180))
                                .lineTo(new Vector2d(-14,-8))
                                .lineTo(new Vector2d(-55,-8))
                                //pick up two white pixels off stack
                                .lineTo(new Vector2d(-14,-8))
                                .lineTo(new Vector2d(30,-8))
                                .splineToConstantHeading(new Vector2d(50,-32),Math.toRadians(0))
                                .turn(0)
                                //place pixels on backboard
                                .splineToConstantHeading(new Vector2d(56,-8), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}