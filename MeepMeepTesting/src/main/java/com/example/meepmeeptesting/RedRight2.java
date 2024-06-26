package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedRight2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, -61, Math.toRadians(180)))
                                //.lineToConstantHeading(new Vector2d(14, -36))
                                //.turn(Math.toRadians(-180))
                                // spit out pixel here
                                //.splineToConstantHeading(new Vector2d(25, -32), Math.toRadians(0))
                                //.splineTo(new Vector2d(46, -32), Math.toRadians(0))
                                //.turn(Math.toRadians(90))
                                // put pixel on board here
                                //.strafeLeft(25)
                                //.back(15)
                                .strafeTo(new Vector2d(50,-32))
                                .turn(0)
                                // place pixel on backboard
                                .strafeRight(10)
                                .lineTo(new Vector2d(15, -22))
                                .back(4)
                                .turn(0)
                                //place pixel on line
                                .lineTo(new Vector2d(24, -8))
                                .lineTo(new Vector2d (-56, -8))
                                //grab pixels off stack
                                .strafeTo(new Vector2d(16,-8))
                                .strafeTo(new Vector2d(38, -8))
                                .splineToConstantHeading(new Vector2d(50,-32), Math.toRadians(0))
                                .turn(0)
                                //place pixel on backboard
                                .splineToConstantHeading(new Vector2d(56,-60), Math.toRadians(0))
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
