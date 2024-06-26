package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeft3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(270)))
                               // .lineToConstantHeading(new Vector2d(-34, -35))
                               // .turn(-1.6)
                                // spit out pixel here
                             //   .lineToConstantHeading(new Vector2d(-34, -11))
                               // .splineToConstantHeading(new Vector2d(-33, -10), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(-16, -10), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(38, -10), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(47, -37), Math.toRadians(0))
                                // place pixel on board here
                                //.strafeRight(25)

                                .lineToLinearHeading(new Pose2d(-34, -25))
                                .turn(0)
                                //place pixel on line
                                .lineTo(new Vector2d(-34,-18))
                                .turn(Math.toRadians(180))
                                .strafeTo(new Vector2d(-34,-8))
                                .lineTo(new Vector2d(-55,-8))
                                .lineTo(new Vector2d(-14, -8))
                                .lineTo(new Vector2d(30,-8))
                                .splineToConstantHeading(new Vector2d(50,-32), Math.toRadians(0))
                                .turn (0)
                                //place pixel on backboard
                                .strafeTo(new Vector2d(30,-8))
                                .lineTo(new Vector2d(-14,-8))
                                .lineTo(new Vector2d(-55,-8))
                                //pick up two white pixels off stack
                                .lineTo(new Vector2d(-14,-8))
                                .lineTo(new Vector2d(30,-8))
                                .splineToConstantHeading(new Vector2d(50,-32),Math.toRadians(0))
                                .turn(0)
                                //place pixels on backboard
                                .lineTo(new Vector2d(47,-32))
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