package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
     .followTrajectorySequence(drive ->
            drive.trajectorySequenceBuilder(new Pose2d(14, 61, Math.toRadians(180)))
                   // .back(5)
                   // .splineToConstantHeading(new Vector2d (25, 26), Math.toRadians(0))
                    // .lineToConstantHeading(new Vector2d(14, 37))
                    // spit out pixel here
                   // .splineToLinearHeading(new Pose2d(50, 32, Math.toRadians(180)), Math.toRadians(0))
                   // .lineToConstantHeading(new Vector2d(47, 34))
                    //. strafeRight(20)
                 // put pixel on board
                    // .splineToConstantHeading(new Vector2d(56, 60), Math.toRadians(0))
                    .strafeTo(new Vector2d(52,36))
                    .turn(0)
                    // place pixel on backboard
                    .lineTo(new Vector2d(20, 32))
                    .turn(0)
                    //place pixel on line
                    .lineTo(new Vector2d(33, 8))
                    .lineTo(new Vector2d (-56, 8))
                    //grab pixels off stack
                    .strafeTo(new Vector2d(16,8))
                    .strafeTo(new Vector2d(38, 8))
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