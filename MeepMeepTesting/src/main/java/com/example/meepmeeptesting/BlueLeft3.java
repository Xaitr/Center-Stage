package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, 65, Math.toRadians(180)))
                              //  .back(1)
                               // .splineToConstantHeading(new Vector2d (16,32), Math.toRadians(0))
                              //  .strafeLeft(24)
                                // spit out pixel here
                             //   .strafeTo(new Vector2d (48, 37))
                              // put pixel on board
                              //  .forward(0.1)
                              //  .splineToConstantHeading(new Vector2d (50, 58), Math.toRadians(0))
                                .strafeTo(new Vector2d(50,32))
                                .turn(0)
                                // place pixel on backboard
                                .lineTo(new Vector2d(10, 32))
                                .turn(0)
                                //place pixel on line
                                .lineToConstantHeading (new Vector2d(15, 32))
                                .splineToConstantHeading(new Vector2d(10,10), Math.toRadians(0))
                                .lineTo(new Vector2d (-56, 10))
                                //grab pixel of stack
                                .strafeTo(new Vector2d(16,10))
                                .strafeTo(new Vector2d(38, 10))
                                .splineToConstantHeading(new Vector2d(50,32), Math.toRadians(0))
                                .turn(0)
                                //place pixel on backboard
                                .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
                                //park






                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}