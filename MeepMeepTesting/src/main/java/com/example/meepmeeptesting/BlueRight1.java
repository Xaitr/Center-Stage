package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRight1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 57, Math.toRadians(180)))
                               // .lineTo(new Vector2d(-36, 30))
                                // spit out pixel here
                              //  .lineTo(new Vector2d(-33, 11))
                                //.splineToSplineHeading(new Pose2d(-32, 10, Math.toRadians(0)), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(-16, 10), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(38, 10), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(50, 34), Math.toRadians(0))
                                // put pixel on board here
                                //.splineToConstantHeading(new Vector2d(58, 14), Math.toRadians(0))
                                .lineTo(new Vector2d(-40, 50))
                                .lineToLinearHeading(new Pose2d(-36, 38, Math.toRadians(90)))
                                .lineTo(new Vector2d(-36,17))
                                .strafeLeft(7)
                                //place pixel on line
                                .back(4)
                                .turn(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-55,13), Math.toRadians(0))
                                //pick up white pixel
                                .splineToConstantHeading (new Vector2d(34, 13), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50,23.5), Math.toRadians(0))
                                .turn(0)
                                //place pixel on backboard
                                .splineToConstantHeading(new Vector2d(30,13), Math.toRadians(180))
                                .lineTo(new Vector2d(-14,13))
                                .lineTo(new Vector2d(-55,13))
                                //pick up two white pixels off stack
                                .lineTo(new Vector2d(-14,13))
                                .lineTo(new Vector2d(30,13))
                                .splineToConstantHeading(new Vector2d(50,34),Math.toRadians(0))
                                .turn(0)
                                //place pixels on backboard
                                .splineToConstantHeading(new Vector2d(50,13), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
