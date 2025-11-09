package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))

                        //TANGENT = 90
                        //FIRST INTAKE
                        .splineTo(new Vector2d(21, 42), Math.PI / 2,
                                new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))


                        //GO TO SMALL TRIANGLE
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(20, 6.7, Math.toRadians(135)), Math.PI / 2,
                                new TranslationalVelConstraint(95), new ProfileAccelConstraint(-50, 50))
                        .splineToSplineHeading(new Pose2d(20, -11, Math.PI), Math.PI / 2,
                                new TranslationalVelConstraint(60), new ProfileAccelConstraint(-33, 33))
//1.5 secs
                        .waitSeconds(3)
                        //MOVE TO 2nd INTAKE POINT

                        .setReversed(false)

                        .splineToSplineHeading(new Pose2d(47, 0, Math.PI / 2), Math.PI,
                                new TranslationalVelConstraint(70), new ProfileAccelConstraint(-50, 50))

                        .splineToConstantHeading(new Vector2d(46, 46.3), Math.PI / 2,
                                new TranslationalVelConstraint(50), new ProfileAccelConstraint(-50, 50))
                        //.splineTo(new Vector2d(44, 47), Math.PI / 2

                        //GO TO SMALL TRIANGLE
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(38, 30), Math.PI / 2,
                                new TranslationalVelConstraint(110), new ProfileAccelConstraint(-75, 75))
                        .splineToSplineHeading(new Pose2d(20, -12, Math.PI), Math.PI / 2,
                                new TranslationalVelConstraint(90), new ProfileAccelConstraint(-60, 60))
//1.5 secs
                        .waitSeconds(3)

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}