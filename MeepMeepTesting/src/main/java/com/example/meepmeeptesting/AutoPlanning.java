package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoPlanning {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(270)))
                                .back(10)
                                .turn(Math.toRadians(45))
                                .turn(Math.toRadians(-90))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(45, -36, Math.toRadians(180)), Math.toRadians(180))
                                .setReversed(false)
                                .back(10)
                                .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}