package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))



                .splineToSplineHeading(new Pose2d(-35, -64, -45), Math.toRadians(-90))

                //FIRST INTAKE

                //.splineTo(new Vector2d(-21, -27), -Math.PI / 2)
                .splineTo(new Vector2d(-42, 0), -Math.PI / 2)

                //GO TO shooting spot(big triangle)
                .splineToSplineHeading(new Pose2d(-35, -64, -45), Math.toRadians(-90))

                .waitSeconds(4)

                //SECOND INTAKE
                .splineTo(new Vector2d(-65, -0), 90)

                //GO TO shooting spot(big triangle)

                .splineToSplineHeading(new Pose2d(70, 0, -45), Math.toRadians(-90))
                .waitSeconds(4)

                .build()
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}