package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueRightTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d (-40, 62, Math.toRadians(90)))
                        .back(2)
                        .splineToConstantHeading(new Vector2d(-48, 25), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-44,15), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(-48,13, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(-54.5, 13), Math.toRadians(180))
                        .lineTo(new Vector2d(30,13))
                        .splineToConstantHeading(new Vector2d(40,36),Math.toRadians(0))
                        .lineTo(new Vector2d(49, 37.5))
                      // below this is backboard to stack 2
                        .splineToConstantHeading(new Vector2d(32, 9), Math.toRadians(180))
                        .lineTo(new Vector2d(-55, 9))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}