package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRight2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, 62, Math.toRadians(90)))
                                .back(2)
                                .splineToConstantHeading(new Vector2d(-48, 25), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(-38,16), Math.toRadians(0))
                                //place pixel on right line
                                .lineToLinearHeading(new Pose2d(-48,13, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-52, 13), Math.toRadians(180))
                                //pick up one white pixel
                                .splineToConstantHeading(new Vector2d(-14,13), Math.toRadians(0))
                                .lineTo(new Vector2d(30,13))
                                .splineToConstantHeading(new Vector2d(50,34),Math.toRadians(0))
                                //place pixel on backboard
                                .lineTo(new Vector2d(30,13))
                                .lineTo(new Vector2d(-14,13))
                                .splineToConstantHeading(new Vector2d(-52, 13), Math.toRadians(180))
                                //pick up two white pixels off stack
                                // .lineTo(new Vector2d(-14,13))
                                .lineTo(new Vector2d(30,13))
                                .splineToConstantHeading(new Vector2d(50,34),Math.toRadians(0))
                                //place pixels on backboard
                                .splineToConstantHeading(new Vector2d(50,13), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(-38,25), Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(-36, 15, Math.toRadians(75)))
//                                //place pixel on line
//                                .splineToLinearHeading(new Pose2d(-36,13), Math.toRadians(180))
//                               // .lineTo(new Vector2d(-36,13))
//                               // .turn(Math.toRadians(105))
//                                .splineToConstantHeading(new Vector2d(-55,13), Math.toRadians(180))
//                                //pick one white pixel off of stack
//                                .lineTo(new Vector2d(20,13))
//                                .splineToConstantHeading(new Vector2d(51,31), Math.toRadians(0))
//                                .turn(0)
//                                //place pixel on backboard
//                                .splineToConstantHeading(new Vector2d(30,13), Math.toRadians(180))
//                                .lineTo(new Vector2d(-14,13))
//                                .lineTo(new Vector2d(-55,13))
//                                //pick up two white pixels off stack
//                                .lineTo(new Vector2d(-14,13))
//                                .lineTo(new Vector2d(30,13))
//                                .splineToConstantHeading(new Vector2d(50,32),Math.toRadians(0))
//                                //place pixels on backboard
//                                .splineToConstantHeading(new Vector2d(48,13), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}