package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft3Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, 65, Math.toRadians(180)))
                                .strafeTo(new Vector2d(52,30))
                                .turn(0)
                                // place pixel on backboard
                                .lineTo(new Vector2d(10, 35))
                                .turn(0)
                                //place pixel on line
                                .lineTo(new Vector2d(33, 57))
                                .lineTo(new Vector2d (-45,57))
                                .turn(0.3)
                                .lineTo(new Vector2d(-55,35))
                                // pick up white pixels off stack
                                .lineTo(new Vector2d(-45,57))
                                .turn(-0.3)
                                .lineTo(new Vector2d(33,57))
                                .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
                                //place two white pixels
                                .lineTo(new Vector2d (-45,57))
                                .turn(0.3)
                                .lineTo(new Vector2d(-55,35))
                                // pick up white pixels off stack
                                .lineTo(new Vector2d(-45,57))
                                .turn(-0.3)
                                .lineTo(new Vector2d(33,57))
                                .splineToConstantHeading(new Vector2d(56,60), Math.toRadians(0))
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